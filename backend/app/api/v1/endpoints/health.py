from fastapi import APIRouter
from api.models import HealthResponse
from services.health_service import HealthService
import logging

logger = logging.getLogger(__name__)

# Create router for health endpoint
router = APIRouter()

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check for the service
    """
    try:
        health_service = HealthService()
        result = health_service.health_check()

        return HealthResponse(
            status=result["status"],
            message=f"Service status: {result['status']}"
        )
    except Exception as e:
        logger.error(f"Error in health check: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            message=f"Service error: {str(e)}"
        )