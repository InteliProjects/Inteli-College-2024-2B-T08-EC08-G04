from fastapi import APIRouter, HTTPException
from controller.controller import RobotNavigator

router = APIRouter()

# Initialize robot_navigator as None and create it during the startup event
robot_navigator = None

@router.on_event("startup")
def initialize_robot_navigator():
    global robot_navigator
    if robot_navigator is None:
        robot_navigator = RobotNavigator(0.0, 0.0, 0.0)

@router.on_event("shutdown")
def shutdown_robot_navigator():
    global robot_navigator
    robot_navigator = None  # Clean up resources if necessary

@router.post("/go_to_position/{position_index}")
async def go_to_position(position_index: int):
    try:
        if robot_navigator is None:
            raise HTTPException(status_code=503, detail="RobotNavigator is not initialized")
        
        result = robot_navigator.go_to_position(position_index)
        return {"status": "success", "result": str(result)}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
