import inspect
import json
from typing import Any, Callable, Literal, Optional

from ollama import chat

MODEL = "qwen3:latest"


def llm_send_move_role(
    robot_id: Literal["pinky1", "pinky2", "pinky3"],
    role_id: Literal["0", "1", "3", "4"],
) -> str:
    """
    Send a move role command to a mobile robot.

    Allowed robot_id (choose exactly one):
    - Even if it is referred to as b44f, robot_id must be entered as pinky1.
    - Even if it is referred to as c0bd, robot_id must be entered as pinky2.
    - Even if it is referred to as 1542, robot_id must be entered as pinky3.

    Allowed role_id (choose exactly one):
    - 0 : WAITING_ZONE
    - 1 : INSPECTION_ZONE
    - 3 : ASSEMBLY_ZONE
    - 4 : MODULE_WAREHOUSE
    """
    if robot_id not in {"pinky1", "pinky2", "pinky3"}:
        return json.dumps({"error": f"invalid robot_id: {robot_id}"}, ensure_ascii=False)
    if role_id not in {"0", "1", "3", "4"}:
        return json.dumps({"error": f"invalid role_id: {role_id}"}, ensure_ascii=False)
    return json.dumps({"robot_id": robot_id, "role_id": role_id}, ensure_ascii=False)


def llm_send_done_by_role(
    role_id: Literal["0", "1", "3", "4"],
    done_type: Literal["load", "unload"],
) -> str:
    """
    Send a done command by role.

    Allowed role_id:
    - 1: 부품 상차 완료일 때
    - 3: 모듈 입고 완료일 때
    - 4: 모듈 상차/출고 완료일 때

    Allowed done_type:
    - load : 상차 완료일 때
    - unload : 하차 완료 또는 출고 완료일 때
    """
    if role_id not in {"0", "1", "3", "4"}:
        return json.dumps({"error": f"invalid role_id: {role_id}"}, ensure_ascii=False)
    if done_type not in {"load", "unload"}:
        return json.dumps({"error": f"invalid done_type: {done_type}"}, ensure_ascii=False)
    return json.dumps({"role_id": role_id, "done_type": done_type}, ensure_ascii=False)


TOOL_REGISTRY = {
    "llm_send_move_role": llm_send_move_role,
    "llm_send_done_by_role": llm_send_done_by_role,
}


def _normalize_tool_arguments(arguments: Any) -> dict:
    # ollama tool arguments 타입이 dict/string 모두 올 수 있어서 정규화
    if arguments is None:
        return {}
    if isinstance(arguments, dict):
        return arguments
    if isinstance(arguments, str):
        try:
            parsed = json.loads(arguments)
            return parsed if isinstance(parsed, dict) else {}
        except json.JSONDecodeError:
            return {}
    return {}


def _validate_tool_call(tool_name: str, arguments: Any) -> dict:
    func = TOOL_REGISTRY.get(tool_name)
    if func is None:
        return {"error": f"unknown tool: {tool_name}"}

    args = _normalize_tool_arguments(arguments)
    sig = inspect.signature(func)
    filtered_args = {k: v for k, v in args.items() if k in sig.parameters}
    result_text = func(**filtered_args)

    try:
        parsed = json.loads(result_text)
        return parsed if isinstance(parsed, dict) else {"error": "invalid_tool_result_format"}
    except json.JSONDecodeError:
        return {"error": "tool_result_parse_error"}


def run_tool_chat(
    prompt: str,
    send_move_role_cb: Optional[Callable[[str, str], None]] = None,
    send_done_by_role_cb: Optional[Callable[[str, str], None]] = None,
) -> dict:
    """
    자연어 -> tool call -> (검증 후) 실제 ROS 실행 콜백 호출

    - send_move_role_cb: gui_node.send_move_role
    - send_done_by_role_cb: main_window.send_done_by_role
    """
    messages = [{"role": "user", "content": prompt}]
    response = chat(MODEL, messages=messages, tools=[llm_send_move_role, llm_send_done_by_role])

    result = {
        "prompt": prompt,
        "tool_called": False,
        "tool_name": None,
        "tool_arguments": {},
        "tool_result": None,
        "final_response": response.message.content,
    }

    if not response.message.tool_calls:
        return result

    tool = response.message.tool_calls[0]
    tool_name = tool.function.name
    tool_args = _normalize_tool_arguments(tool.function.arguments)
    validation = _validate_tool_call(tool_name, tool_args)

    dispatch = {"dispatched": False}
    if "error" not in validation:
        if tool_name == "llm_send_move_role" and send_move_role_cb is not None:
            send_move_role_cb(validation["robot_id"], validation["role_id"])
            dispatch = {"dispatched": True}
        elif tool_name == "llm_send_done_by_role" and send_done_by_role_cb is not None:
            send_done_by_role_cb(validation["role_id"], validation["done_type"])
            dispatch = {"dispatched": True}

    tool_result = {"validation": validation, "dispatch": dispatch}
    tool_result_text = json.dumps(tool_result, ensure_ascii=False)

    messages.append(response.message)
    messages.append({"role": "tool", "content": tool_result_text})
    final = chat(MODEL, messages=messages)

    result.update(
        {
            "tool_called": True,
            "tool_name": tool_name,
            "tool_arguments": tool_args,
            "tool_result": tool_result,
            "final_response": final.message.content,
        }
    )
    return result
