from .rag_agent import (
    greeting_agent,
    rag_agent,
    selected_text_agent,
    main_agent
)

# Import the required classes from the OpenAI Agents SDK
try:
    from agents import (
        Agent, Runner, OpenAIChatCompletionsModel,
        RunContextWrapper, function_tool, trace, ModelSettings
    )
    # Re-export them so they can be imported from agent_defs
    __all__ = [
        'Agent', 'Runner', 'OpenAIChatCompletionsModel', 'RunContextWrapper',
        'function_tool', 'trace', 'ModelSettings',
        'greeting_agent', 'rag_agent', 'selected_text_agent', 'main_agent'
    ]
except ImportError:
    # Define placeholder classes if the agent library is not available yet
    # This is just for development purposes
    Agent = None
    Runner = None
    OpenAIChatCompletionsModel = None
    RunContextWrapper = None
    function_tool = None
    trace = None
    ModelSettings = None
    __all__ = [
        'Agent', 'Runner', 'OpenAIChatCompletionsModel', 'RunContextWrapper',
        'function_tool', 'trace', 'ModelSettings',
        'greeting_agent', 'rag_agent', 'selected_text_agent', 'main_agent'
    ]
