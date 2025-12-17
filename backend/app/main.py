from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.v1.router import api_router
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="FastAPI backend for RAG-powered chatbot using OpenAI Agents SDK",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "http://127.0.0.1:3000",
        "https://docusaurus-project-1090598679780.us-central1.run.app",  # Deployed frontend
        "*",  # Allow all origins (remove in production if you want strict control)
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# Include API routes
app.include_router(api_router, prefix="/api/v1")

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)