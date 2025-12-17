// ../types/chat.ts

export interface Message {
  id: string;
  content: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  status: 'sent' | 'delivered' | 'error';
}

export interface ChatSession {
  id: string;
  messages: Message[];
  createdAt: Date;
  lastActiveAt: Date;
}

export interface ChatState {
  currentSession: ChatSession | null;
  isLoading: boolean;
  error: string | null;
  isVisible: boolean;
}

export interface ChatRequest {
  message: string;
  sessionId?: string;
  pageContext?: {
    url: string;
    title: string;
    path: string;
  };
}

export interface SelectedTextChatRequest extends ChatRequest {
  selectedText: string;
}

export interface BackendResponse {
  answer: string;         // backend response ka main message
  thread_id?: string;     // optional session/thread id
  sources?: any[];        // optional sources
}

export interface ApiError {
  error: string;
}
