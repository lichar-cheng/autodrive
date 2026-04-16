import uvicorn
from app.config import CONFIG


if __name__ == "__main__":
    uvicorn.run("app.main:app", host=CONFIG.host, port=CONFIG.port, reload=False)
