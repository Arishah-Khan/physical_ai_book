import time
import logging
from typing import Callable, Awaitable
from fastapi import Request, Response
from fastapi.responses import JSONResponse
import json

# Set up logger for middleware
logger = logging.getLogger(__name__)

async def logging_middleware(request: Request, call_next: Callable[[Request], Awaitable[Response]]):
    """
    Middleware to log request/response information
    """
    start_time = time.time()

    # Log request
    logger.info(f"Request: {request.method} {request.url}")
    if request.query_params:
        logger.info(f"Query params: {dict(request.query_params)}")

    try:
        response = await call_next(request)

        # Calculate process time
        process_time = time.time() - start_time

        # Log response
        logger.info(f"Response: {response.status_code} in {process_time:.2f}s")

        # Add process time to response headers
        response.headers["X-Process-Time"] = str(process_time)

        return response
    except Exception as e:
        process_time = time.time() - start_time
        logger.error(f"Request failed: {request.method} {request.url} - {str(e)} in {process_time:.2f}s")
        raise


async def error_handler_middleware(request: Request, call_next: Callable[[Request], Awaitable[Response]]):
    """
    Middleware to handle errors and return consistent error responses
    """
    try:
        response = await call_next(request)
        return response
    except Exception as exc:
        logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
        return JSONResponse(
            status_code=500,
            content={"error": "Internal server error"}
        )