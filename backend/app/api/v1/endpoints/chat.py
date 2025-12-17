from fastapi import APIRouter, HTTPException
from typing import Dict, Any
from app.api.v1.endpoints.models import ChatRequest, ChatResponse, SelectedTextChatRequest
from agents import Runner, trace,enable_verbose_stdout_logging
from agent_defs.rag_agent import main_agent, selected_text_agent
import logging

logger = logging.getLogger(__name__)

# Create router for chat endpoints
router = APIRouter()
enable_verbose_stdout_logging()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Process user query using RAG functionality with OpenAI Agents
    """
    try:
        # Use OpenAI Agents SDK to process the request
        with trace("RAG Chat"):
            result = await Runner.run(
                main_agent,
                request.message
            )

            # Extract the response from the result
            answer = result.final_output if hasattr(result, 'final_output') else str(result)

        # Format the response using the contract model
        response = ChatResponse(
            answer=answer,
            thread_id=request.thread_id,
            sources=[]  # Sources will be populated based on agent response
        )

        return response
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in /chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/chat/selected-text", response_model=ChatResponse)
async def chat_selected_text_endpoint(request: SelectedTextChatRequest):
    """
    Process user query based only on provided selected text
    """
    try:
        # Create a combined prompt that includes the selected text
        prompt = f"Based on the following text: '{request.selected_text}', please answer this question: {request.message}"

        # Use the selected_text_agent to process the request
        with trace("Selected Text Chat"):
            result = await Runner.run(
                selected_text_agent,
                prompt
            )

            # Extract the response from the result
            answer = result.final_output if hasattr(result, 'final_output') else str(result)

        # Format the response using the contract model
        response = ChatResponse(
            answer=answer,
            thread_id=request.thread_id,
            sources=[]  # No external sources for selected text agent
        )

        return response
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in /chat/selected-text endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")