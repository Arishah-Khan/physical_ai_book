/**
 * Chat API Service
 * Handles communication with backend chat endpoints
 */

import type {
  ChatRequest,
  SelectedTextChatRequest,
  BackendResponse,
  ApiError,
} from '../types/chat';

// Get the API base URL from environment or default to localhost
const API_BASE_URL = process.env.CHAT_API_BASE_URL || 'http://localhost:8000';

// Log the API URL for debugging (remove in production)
console.log('Chat API Base URL:', API_BASE_URL);

/**
 * Send a general chat query to the backend
 */
export async function sendChatMessage(
  request: ChatRequest
): Promise<BackendResponse> {
  try {
    // Transform to match backend expected format
    const payload = {
      message: request.message,
      thread_id: request.sessionId || undefined,
      page_context: request.pageContext,
    };

    const response = await fetch(`${API_BASE_URL}/api/v1/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    });

    if (!response.ok) {
      const errorData: ApiError = await response.json();
      throw new Error(errorData.error || `HTTP error ${response.status}`);
    }

    const data: BackendResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Error sending chat message:', error);
    throw error;
  }
}

/**
 * Send a query about selected text to the backend
 */
export async function sendSelectedTextMessage(
  request: SelectedTextChatRequest
): Promise<BackendResponse> {
  try {
    // Transform to match backend expected format (snake_case)
    const payload = {
      message: request.message,
      selected_text: request.selectedText,  // ✅ Changed from selectedText to selected_text
      thread_id: request.sessionId || undefined,  // ✅ Changed from sessionId to thread_id
      page_context: request.pageContext,
    };

    const response = await fetch(`${API_BASE_URL}/api/v1/chat/selected-text`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    });

    if (!response.ok) {
      const errorData: ApiError = await response.json();
      throw new Error(errorData.error || `HTTP error ${response.status}`);
    }

    const data: BackendResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Error sending selected text message:', error);
    throw error;
  }
}

/**
 * Get the current page context from the browser
 */
export function getPageContext() {
  return {
    url: window.location.href,
    title: document.title,
    path: window.location.pathname,
  };
}

/**
 * Capture selected text from the page
 */
export function getSelectedText(): string | null {
  const selection = window.getSelection();
  return selection ? selection.toString().trim() : null;
}