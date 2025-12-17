import os

from agents import (
    Agent,OpenAIChatCompletionsModel,AsyncOpenAI
)
from .tools import search_documentation, greet_user,search_qdrant
import logging
from config.settings import settings
from dotenv import load_dotenv

load_dotenv()

api_key = os.getenv("GEMINI_API_KEY")

client = AsyncOpenAI(
    api_key=api_key,
    base_url= "https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model = "gemini-2.5-flash",
    openai_client=client
)

# ============================================
# 1. GREETING AGENT
# ============================================
greeting_agent = Agent[None](
    name="Greeting Assistant",
    instructions="""
You are a friendly greeting assistant.
Your ONLY job is to handle greetings like: "hello", "hi", "hey", "good morning", etc.

👉 When you detect a greeting, call the greet_user tool.
👉 Do NOT answer documentation questions - those go to the RAG Agent.
""",
    tools=[greet_user],
    model=model
)

# ============================================
# 2. RAG AGENT (Main documentation Q&A)
# ============================================
rag_agent = Agent[None](
    name="Documentation RAG Agent",
    instructions="""
You are a helpful documentation assistant that answers questions about the book content.

Your workflow:
1. When a user asks a question, ALWAYS use the search_documentation tool first
2. Analyze the retrieved content carefully
3. Provide a clear, accurate answer based on the search results
4. Cite the source files when providing information
5. If no results found, ask user to rephrase

Guidelines:
- Always search before answering
- Be precise and factual - only use information from search results
- Don't make up information
- Reference specific files/sections when relevant
""",
    tools=[search_documentation],
    model=model
)

# ============================================
# 3. SELECTED TEXT AGENT (No RAG)
# ============================================
selected_text_agent = Agent[None](
    name="Selected Text Assistant",
    instructions="""
You are a helpful assistant that answers questions based only on the provided text context.
    Do not use any external knowledge or documentation - only answer based on the text provided by the user.
    If the answer cannot be found in the provided text, acknowledge this limitation.
""",
    model=model,
    tools=[search_documentation]
)

# ============================================
# 4. MAIN ROUTER AGENT (Entry point)
# ============================================
main_agent = Agent[None](
    name="Main Documentation Assistant",
    instructions="""
You are the main documentation assistant router.

Route queries to appropriate agent:
1. If user greets (hello, hi, hey, etc.) → HANDOFF to Greeting Assistant
2. If user asks about book/documentation → HANDOFF to Documentation RAG Agent

Always handoff to the appropriate agent. Do not answer yourself.
""",
    model=model,
    handoffs=[search_qdrant,greeting_agent, rag_agent]
)