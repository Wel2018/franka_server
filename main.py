import sys
sys.path.append("../")
import platform
import signal
import uvicorn
import logging
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI
# from fastapi.responses import *
# from fastapi import *
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from rich import print
import multiprocessing as mp
import asyncio
from config import AppConfig, get_origins


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


# uvicorn 模式下，app 必须放在全局作用域中，不能放在 main 函数中！
app = create_app(get_origins(1))

# 挂载静态文件目录，模拟静态资源服务器
# app.mount("/tmp", StaticFiles(directory="tmp"), name="static")

# 挂载路由
from api import router as franka_api
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


def main(args=[]):
    # print("启动 http 服务...")
    arm_id = args[0]
    if arm_id == "2":
        port = 29000
    elif arm_id == "3":
        port = 29001
    else:
        raise ValueError(f"不支持设备ID = {arm_id}") 
    
    cfg = AppConfig(
        "main:app",
        is_encrypt=0,
        port=port,
    )

    cfg.args['arm_id'] = arm_id
    # print(f"cfg={cfg}")
    # from api import router as franka_api
    import api
    api.create_arm()
    run_uvcorn(cfg)
    # print("关闭服务...")


if __name__ == '__main__':
    import sys
    print("--- Franka Server ---")
    print("使用方法：python main.py [2|3] \n\n")
    if len(sys.argv) < 2:
        print("请按照使用方法使用")
        exit(1)
    
    args = sys.argv[1:]
    main(args)
