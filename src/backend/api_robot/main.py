from fastapi import FastAPI
from app.routes import router
import rclpy

# Initialize rclpy before starting the app
rclpy.init()

app = FastAPI()

# Include the routes
app.include_router(router)

# Shutdown rclpy when the application stops
@app.on_event("shutdown")
def shutdown_rclpy():
    rclpy.shutdown()
