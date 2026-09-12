# 不要动该文件！！！
# 不要动该文件！！！
# 不要动该文件！！！

BASE_URL = "http://127.0.0.1:2026"
ROBOT_ID = "202609044048"

# 请求和返回
import json
import time
import uuid
from http.client import IncompleteRead, RemoteDisconnected
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

MAX_REQUEST_ATTEMPTS = 3
RETRY_DELAY_S = 0.2


def post(path, data):

    # json转换
    json_data = json.dumps(data).encode("utf-8")

    for attempt in range(1, MAX_REQUEST_ATTEMPTS + 1):
        request = Request(
            BASE_URL + path,
            data=json_data,
            headers={
                "Content-Type": "application/json"
            },
            method="POST"
        )
        # 自动重发
        try:
            with urlopen(request, timeout=5) as response:
                return json.loads(
                    response.read().decode("utf-8")
                )
        except HTTPError:
            raise
        except (
            URLError,
            TimeoutError,
            ConnectionError,
            RemoteDisconnected,
            IncompleteRead,
            json.JSONDecodeError,
            UnicodeDecodeError,
        ):
            if attempt == MAX_REQUEST_ATTEMPTS:
                raise

            time.sleep(RETRY_DELAY_S * 2 ** (attempt - 1))

# 自动配发request_id
def _base_payload():
    return {
        "arena_id": "default",
        "robot_id": ROBOT_ID,
        "request_id": str(uuid.uuid4())
    }

def enter():

    # /enter 开始模拟
    return post("/enter", _base_payload())

def measure(x, y, channel):

    # /measure方法
    payload = _base_payload()
    payload["position"] = {"x": x, "y": y}
    payload["channel"] = channel
    return post("/measure", payload)

def clear(x, y, channel):

    # /clear方法
    payload = _base_payload()
    payload["position"] = {"x": x, "y": y}
    payload["channel"] = channel
    return post("/clear", payload)

def exit_robot():

    # /exit 结束模拟
    return post("/exit", _base_payload())

# 不要动该文件！！！
# 不要动该文件！！！
# 不要动该文件！！！