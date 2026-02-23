from flask import Flask, render_template, request, redirect, url_for, send_file
import pandas as pd
import mysql.connector
import io
import pdfplumber
import re 
import time
import os

# ==========================================
# ★ [수정] 폴더 경로 설정 (Backend/Frontend 분리 대응)
base_dir = os.path.abspath(os.path.dirname(__file__))

# 템플릿(HTML) 경로: backend 폴더에서 한 단계 위(..)로 가서 frontend/templates 찾기
template_dir = os.path.join(base_dir, '../frontend/templates')

# 정적(이미지/CSS) 경로: backend 폴더에서 한 단계 위(..)로 가서 frontend/static 찾기
static_dir = os.path.join(base_dir, '../frontend/static')

# Flask 앱 생성 시 template_folder와 static_folder 위치를 지정
app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)

# ==========================================
# 우분투 노트북 DB 접속 정보
db_config = {
    # 우분투 터미널에서 'hostname -I'로 확인한 IP 주소로 변경
    'host': '192.168.0.25',
    'user': 'root',
    'password': '1234',      
    'database': 'smart_factory'
}

@app.route('/')
def home():
    return render_template('index.html')

# [핵심 기능 0] PDF 모델명 번역
def translate_pdf_name(pdf_raw_name):
    key = str(pdf_raw_name).strip().upper().replace(" ", "")
    mapping = {
        "45ELBOWCL2E0A0NA":   {"name": "45 ELBOW CLEAN PVC", "size": "200A"},
        "45ELBOWST2S0300A4":  {"name": "45 ELBOW STS304 AP SCH10S BW", "size": "200A"},
        "45ELBOWST2S5300A4":  {"name": "45 ELBOW STS304 AP SCH10S BW", "size": "250A"},
        "45ELBOWST4S0300A4":  {"name": "45 ELBOW STS304 AP SCH10S BW", "size": "400A"},
        "45ELBOWST4S5300A4":  {"name": "45 ELBOW STS304 AP SCH10S BW", "size": "450A"},
        "45ELBOWSP1P0G0AA":   {"name": "45 ELBOW SPP GAL'V BW", "size": "100A"}
    }
    return mapping.get(key, None)

# [핵심 기능 1] 엑셀용 이름 표준화
def standardize_excel_name(raw_name):
    clean_raw = str(raw_name).strip()
    name_key = clean_raw.upper().replace(" ", "")
    aliases = {
        "45EL": "45 ELBOW", "90EL": "90 ELBOW",
        "45ELBOW": "45 ELBOW", "ELBOW": "45 ELBOW",
        "TEE": "EQUAL TEE", "SOCKET": "SOCKET", "VALVE": "GATE VALVE"
    }
    return aliases.get(name_key, clean_raw)

# [핵심 기능 2] 스마트 DB 매칭
def find_best_match(cursor, name_core, size_val):
    full_name_candidate = f"{name_core} ({size_val})"
    cursor.execute("SELECT part_id FROM parts WHERE part_name = %s", (full_name_candidate,))
    row = cursor.fetchone()
    if row: return row[0]

    cursor.execute("""
        SELECT part_id FROM parts 
        WHERE part_name LIKE %s AND part_name LIKE %s 
        ORDER BY part_id ASC LIMIT 1
    """, (f"%{name_core}%", f"%{size_val}%"))
    row = cursor.fetchone()
    if row: return row[0]
    return None

# [PDF 처리]
def parse_pdf_to_df(file_obj):
    table_settings = { "vertical_strategy": "text", "horizontal_strategy": "text", "intersection_x_tolerance": 5 }
    with pdfplumber.open(file_obj) as pdf:
        all_rows = []
        for page in pdf.pages:
            tables = page.extract_tables(table_settings)
            for table in tables:
                for row in table:
                    cleaned_row = [str(cell).strip() if cell is not None else '' for cell in row]
                    if any(cell for cell in cleaned_row if cell): all_rows.append(cleaned_row)
    if not all_rows: return None
    
    header_index = -1
    for i, row in enumerate(all_rows):
        row_str = "".join(row).upper()
        if "DESC" in row_str or "QTY" in row_str:
            header_index = i
            break   
    if header_index == -1: header = all_rows[0]; data = all_rows[1:]
    else: header = all_rows[header_index]; data = all_rows[header_index + 1:]

    clean_data = []
    for row in data:
        if len(row) == len(header): clean_data.append(row)
        elif len(row) > len(header): clean_data.append(row[:len(header)]) 
        else: clean_data.append(row + [''] * (len(header) - len(row))) 
    df = pd.DataFrame(clean_data, columns=header)
    return df

# [스마트 컬럼]
def smart_assign_columns(df):
    new_cols = []
    for c in df.columns:
        c_str = str(c).strip().upper()
        if "DESC" in c_str: new_cols.append("DESCRIPTION")
        elif "QTY" in c_str or "QUAN" in c_str: new_cols.append("QTY")
        elif "SIZE" in c_str: new_cols.append("SIZE")
        else: new_cols.append(c_str)
    df.columns = new_cols
    if "DESCRIPTION" in df.columns and "QTY" not in df.columns:
        cols = list(df.columns); cols[-1] = "QTY"; df.columns = cols
    if "DESCRIPTION" not in df.columns:
        max_idx = -1; max_len = 0
        for i, col in enumerate(df.columns):
            if col == "QTY": continue
            curr_len = sum(len(str(x)) for x in df[col])
            if curr_len > max_len: max_len = curr_len; max_idx = i
        if max_idx != -1: cols = list(df.columns); cols[max_idx] = "DESCRIPTION"; df.columns = cols
    return df

# [업로드 로직 - 주문 누적 & ID 재사용]
@app.route('/upload_excel', methods=['POST'])
def upload_excel():
    user_name = request.form['username']
    project_name = request.form['project']
    file = request.files['excel_file']
    if not file: return "파일이 없습니다."

    filename = file.filename.lower()
    df = None
    
    if filename.endswith(('.xlsx', '.xls')): df = pd.read_excel(file).fillna(0)
    elif filename.endswith('.pdf'): 
        df = parse_pdf_to_df(file)
        if df is None: return "<h3>PDF 인식 실패</h3>"
    else: return "<h3>지원하지 않는 파일 형식</h3>"

    df.columns = df.columns.str.replace('\n', '').str.strip()
    df = smart_assign_columns(df) 

    # 데드락 방지용 재시도 로직
    max_retries = 3
    for attempt in range(max_retries):
        conn = None
        try:
            conn = mysql.connector.connect(**db_config)
            cursor = conn.cursor()

            # 1. 새 주문(Quotes) 생성
            sql_header = "INSERT INTO quotes (project_name, customer_name, status, created_at) VALUES (%s, %s, 'SAVED', NOW())"
            cursor.execute(sql_header, (project_name, user_name))
            new_quote_id = cursor.lastrowid 
            saved_count = 0

            # 2. 엑셀 데이터 처리
            for index, row in df.iterrows():
                raw_desc = str(row['DESCRIPTION']).strip()
                translation = translate_pdf_name(raw_desc)
                if translation:
                    desc = translation['name']; size = translation['size']
                else:
                    desc = standardize_excel_name(raw_desc)
                    size = str(row['SIZE']).strip() if 'SIZE' in df.columns else '0'

                if size not in ['0', '', 'None', 'nan', '-']: full_name = f"{desc} ({size})"
                else: full_name = desc; size = "-"

                try: 
                    qty_val = str(row['QTY']).replace(',', '').split('.')[0].strip()
                    excel_qty = int(qty_val) if qty_val else 0
                except: excel_qty = 0
                if excel_qty <= 0: excel_qty = 1 

                # 스마트 ID 로직
                # DB에 이미 있는 부품인지 확인
                real_id = find_best_match(cursor, desc, size)
                
                if not real_id:
                    # 없으면 새로 생성 (기존 최대값 + 1)
                    cursor.execute("SELECT MAX(CAST(part_id AS UNSIGNED)) FROM parts")
                    max_row = cursor.fetchone()
                    try: current_max = int(max_row[0]) if (max_row and max_row[0]) else 0
                    except: current_max = 0
                    real_id = str(current_max + 1)
                    # parts 테이블에 등록
                    cursor.execute("INSERT INTO parts (part_id, part_name, spec, unit_price, location_name, current_stock) VALUES (%s, %s, '자동등록', 0, '대기존', 1000)", (real_id, full_name))
                
                # 주문 상세 내역 저장 
                cursor.execute("INSERT INTO quote_details (quote_id, part_id, req_quantity) VALUES (%s, %s, %s)", (new_quote_id, real_id, excel_qty))
                
                # 재고 차감
                cursor.execute("UPDATE parts SET current_stock = current_stock - %s WHERE part_id = %s", (excel_qty, real_id))
                saved_count += 1

            conn.commit()
            return render_template('success.html', count=saved_count, quote_id=new_quote_id, project=project_name)

        except mysql.connector.Error as err:
            if err.errno == 1213: # Deadlock
                print(f"⚠️ 데드락 발생! 재시도 중... ({attempt+1}/{max_retries})")
                time.sleep(1)
                if conn: conn.rollback()
                continue 
            else:
                if conn: conn.rollback()
                return f"<h1>오류 발생</h1><p>{err}</p><br><a href='/'>돌아가기</a>"
        finally:
            if conn: conn.close()
    
    return "<h1>시스템 혼잡</h1><p>잠시 후 다시 시도해주세요. (Deadlock Retry Failed)</p>"

# 뷰/다운로드/목록/삭제 로직은 기존과 동일
@app.route('/view_order/<quote_id>')
def view_order(quote_id):
    rows, info = get_quote_data(quote_id)
    if not info: return "데이터 없음"
    return render_template('result.html', username=info['customer_name'], project=info['project_name'], rows=rows, quote_id=quote_id, date=info['created_at'].strftime("%Y-%m-%d"))

@app.route('/download_excel/<quote_id>')
def download_excel(quote_id):
    rows, info = get_quote_data(quote_id)
    if not rows: return "데이터 없음"
    export_list = []
    for r in rows: export_list.append({"ArUco ID": r['id'], "부품명": r['name'], "규격": r['size'], "수량": r['qty']})
    df = pd.DataFrame(export_list)
    output = io.BytesIO()
    with pd.ExcelWriter(output, engine='openpyxl') as writer:
        df.to_excel(writer, index=False); writer.sheets['Sheet1'].column_dimensions['B'].width = 30
    output.seek(0)
    return send_file(output, as_attachment=True, download_name=f"Quote_{quote_id}.xlsx", mimetype="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet")

@app.route('/list')
def order_list():
    conn = mysql.connector.connect(**db_config); cursor = conn.cursor(dictionary=True)
    cursor.execute("SELECT * FROM quotes ORDER BY quote_id DESC"); quotes = cursor.fetchall(); conn.close()
    return render_template('order_list.html', quotes=quotes)

@app.route('/delete_order/<int:quote_id>')
def delete_order(quote_id):
    conn = mysql.connector.connect(**db_config); cursor = conn.cursor()
    cursor.execute("DELETE FROM quote_details WHERE quote_id = %s", (quote_id,))
    cursor.execute("DELETE FROM quotes WHERE quote_id = %s", (quote_id,))
    conn.commit(); conn.close()
    return redirect(url_for('order_list'))

def get_quote_data(quote_id):
    conn = None
    try:
        conn = mysql.connector.connect(**db_config); cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT * FROM quotes WHERE quote_id = %s", (quote_id,)); info = cursor.fetchone()
        cursor.execute("SELECT p.part_id, p.part_name, qd.req_quantity FROM quote_details qd JOIN parts p ON qd.part_id = p.part_id WHERE qd.quote_id = %s", (quote_id,))
        items = cursor.fetchall()
        display_rows = []
        for item in items:
            full = item['part_name']
            if '(' in full and full.endswith(')'): parts = full.split('('); name = parts[0].strip(); size = parts[1].replace(')', '').strip()
            else: name = full; size = "-"
            display_rows.append({"id": item['part_id'], "name": name, "size": size, "qty": item['req_quantity']})
        return display_rows, info
    except: return [], None
    finally: 
        if conn: conn.close()

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)