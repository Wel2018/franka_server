# 配置文件
from dataclasses import dataclass, asdict


@dataclass
class AppConfig:
    args = {}

    # uvicorn 应用名
    uvicorn_app_url: str = "backend.main:app"
    # 服务器地址
    host: str = "0.0.0.0"
    port: int = 29000
    
    # [状态位]
    # 是否需要支持跨域访问
    is_cors: int = 1
    # 是否支持任意访问者
    is_anyone: int = 1
    # 是否对接口使用 HTTPS 加密
    is_encrypt: int = 0
    # 是否为部署方式
    is_deploy: int = 0

    @property
    def hostname(self):
        prefix = "https" if self.is_encrypt else "http"
        return f"{prefix}://127.0.0.1:{self.port}/docs"

    def to_dict(self):
        return asdict(self)


# 跨域访问
# https://fastapi.tiangolo.com/zh/tutorial/cors/
origins = [
    # '*', # 任意源
    "http://localhost",
    "https://localhost",
    "http://localhost:8080",
    "http://127.0.0.1:3000",
    "http://localhost:3000",
    "http://localhost:28100",
    "http://localhost:29000",
    "https://192.168.31.11:8100",
    "https://192.168.31.11:8080",
]

local_ports = [8080, 3000, 8000]


def get_origins(is_anyone=1):
    """后端服务统一使用 http 不加密版本，不配置跨域访问"""
    # 需要在 host 0.0.0.0 启动
    if is_anyone:
        return ["*"]
    else:
        return origins
