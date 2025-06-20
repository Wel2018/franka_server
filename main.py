import sys
sys.path.append("../")
import platform
import signal
import uvicorn
import logging
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from rich import print
import multiprocessing as mp
import asyncio
from config import AppConfig, get_origins
from api import router as franka_api


def create_app(origins: list):
    """创建全局唯一实例"""
    #app = FastAPI(on_startup=[on_startup])
    app = FastAPI()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        # allow_origin_regex="*",
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    return app


# 普通服务
app = create_app(get_origins(1))
# 挂载静态文件目录，模拟静态资源服务器
# app.mount("/tmp", StaticFiles(directory="tmp"), name="static")
app.include_router(franka_api, prefix="/ctl", tags=["franka api"])


# 启动服务器
def run_uvcorn(cfg: AppConfig):
    # print(config.to_dict())
    print(f"服务地址：{cfg.hostname}")
    uvicorn.run(
        cfg.uvicorn_app_url,
        host=cfg.host,
        port=cfg.port, 
        # reload=bool(0),
        # reload_dirs="./backend",
        # reload_delay=3, 
        log_level=logging.WARN,
        # ssl_keyfile=cfg.ssl_keyfile,
        # ssl_certfile=cfg.ssl_certfile,
    )


def main(args):
    print("启动 http 服务...")
    cfg = AppConfig(
        "main:app",
        is_encrypt=0,
    )
    cfg.args = args
    run_uvcorn(cfg)
    print("关闭服务...")


if __name__ == '__main__':
    import sys
    args = sys.argv[1:]
    main(args)
