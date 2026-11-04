"""Runtime map configuration for the frontend."""
from typing import Optional

from fastapi import APIRouter, Response

from gcs.backend.map_config import cesium_ion_token

router = APIRouter()


@router.get("/api/map-config")
async def get_map_config(response: Response) -> dict[str, Optional[str]]:
    # Read on every request, so a token file added later needs no restart.
    response.headers["Cache-Control"] = "no-store"
    return {"cesium_ion_token": cesium_ion_token()}
