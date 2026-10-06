"""Health check endpoint."""
from fastapi import APIRouter, Request
from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.broadcast import manager as ws_manager

router = APIRouter()


@router.get("/health")
async def health(request: Request):
    return {
        "status": "ok",
        "vehicles_connected": len(vehicle_mgr.vehicles),
        "ws_clients": ws_manager.client_count,
        "unavailable_routes": list(getattr(request.app.state, "unavailable_routes", ())),
    }
