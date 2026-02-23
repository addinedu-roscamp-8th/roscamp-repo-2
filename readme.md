app.py -> 25번줄
db_config = {
    # 우분투 터미널에서 'hostname -I'로 확인한 IP 주소로 변경
    'host': '192.168.0.25',
    'user': 'root',
    'password': '1234',      
    'database': 'smart_factory'
}

cd backend
python3 app.py

web 검색창에 127.0.0.1:5000 입력

