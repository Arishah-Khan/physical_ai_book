/**
 * BookChatbot Component
 * Reusable chatbot component for the Docusaurus documentation site
 */

import React, { useState, useEffect, useRef } from 'react';
import type { Message, ChatSession, ChatState, BackendResponse } from '../types/chat';
import { sendChatMessage, sendSelectedTextMessage, getPageContext } from '../services/chatApi';
import { saveSession, loadSession, clearSession, isSessionValid } from '../utils/sessionStorage';
import './BookChatbot.css';

const BookChatbot: React.FC = () => {
  const [chatState, setChatState] = useState<ChatState>({
    currentSession: null,
    isLoading: false,
    error: null,
    isVisible: false,
  });

  const [selectedText, setSelectedText] = useState<string | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Scroll chat to bottom when messages change
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [chatState.currentSession?.messages]);

  // Toggle chat window
  const toggleChat = () => {
    setChatState(prev => ({ ...prev, isVisible: !prev.isVisible }));
    if (!chatState.isVisible) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  // Clear chat history
  const handleClearHistory = () => {
    if (confirm('Are you sure you want to clear the chat history?')) {
      clearSession();
      const newSession: ChatSession = {
        id: `session-${Date.now()}`,
        messages: [],
        createdAt: new Date(),
        lastActiveAt: new Date(),
      };
      setChatState(prev => ({ ...prev, currentSession: newSession }));
      saveSession(newSession);
    }
  };

  // Load or initialize session
  useEffect(() => {
    if (!chatState.currentSession) {
      const storedSession = loadSession();
      if (storedSession && isSessionValid(storedSession)) {
        setChatState(prev => ({ ...prev, currentSession: storedSession }));
      } else {
        const newSession: ChatSession = {
          id: `session-${Date.now()}`,
          messages: [],
          createdAt: new Date(),
          lastActiveAt: new Date(),
        };
        setChatState(prev => ({ ...prev, currentSession: newSession }));
        saveSession(newSession);
      }
    }
  }, [chatState.currentSession]);

  // Save session on updates
  useEffect(() => {
    if (chatState.currentSession) saveSession(chatState.currentSession);
  }, [chatState.currentSession]);

  // Handle Escape key to close chat
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && chatState.isVisible) toggleChat();
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [chatState.isVisible]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection()?.toString().trim() || null;
      if (text && text.length > 5) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  // Handle message submission
  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const input = e.currentTarget.elements.namedItem('message') as HTMLInputElement;
    const messageText = input.value.trim();
    if (!messageText || chatState.isLoading) return;

    input.value = '';

    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      content: messageText,
      sender: 'user',
      timestamp: new Date(),
      status: 'sent',
    };

    setChatState(prev => ({
      ...prev,
      currentSession: prev.currentSession
        ? { ...prev.currentSession, messages: [...prev.currentSession.messages, userMessage], lastActiveAt: new Date() }
        : null,
    }));

    setChatState(prev => ({ ...prev, isLoading: true, error: null }));

    try {
      const pageContext = getPageContext();

      const response: BackendResponse = selectedText
        ? await sendSelectedTextMessage({
            message: messageText,
            selectedText,
            sessionId: chatState.currentSession?.id,
            pageContext,
          })
        : await sendChatMessage({
            message: messageText,
            sessionId: chatState.currentSession?.id,
            pageContext,
          });

      const botMessage: Message = {
        id: `msg-${Date.now()}`,
        content: response.answer,
        sender: 'bot',
        timestamp: new Date(),
        status: 'delivered',
      };

      setChatState(prev => ({
        ...prev,
        isLoading: false,
        currentSession: prev.currentSession
          ? {
              ...prev.currentSession,
              id: response.thread_id || prev.currentSession.id,
              messages: [...prev.currentSession.messages, botMessage],
              lastActiveAt: new Date(),
            }
          : null,
      }));

      // Clear selected text after sending
      setSelectedText(null);
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to send message';
      setChatState(prev => ({ ...prev, isLoading: false, error: errorMessage }));

      const errorBotMessage: Message = {
        id: `msg-${Date.now()}`,
        content: `Sorry, I encountered an error: ${errorMessage}. Please try again.`,
        sender: 'bot',
        timestamp: new Date(),
        status: 'error',
      };

      setChatState(prev => ({
        ...prev,
        currentSession: prev.currentSession
          ? { ...prev.currentSession, messages: [...prev.currentSession.messages, errorBotMessage], lastActiveAt: new Date() }
          : null,
      }));
    }
  };

  return (
    <>
      <button
        className="chat-toggle-button"
        onClick={toggleChat}
        aria-label="Toggle chat"
        title={chatState.isVisible ? 'Close chat' : 'Open chat'}
      >
        {chatState.isVisible ? '✕' : '💬'}
      </button>

      {chatState.isVisible && (
        <div className="chat-window" role="dialog" aria-label="Documentation Assistant Chat">
          <div className="chat-header">
            <h3 id="chat-title">Documentation Assistant</h3>
            <div className="chat-header-actions">
              <button className="chat-clear-button" onClick={handleClearHistory} aria-label="Clear chat history" title="Clear chat history">🗑️</button>
              <button className="chat-close-button" onClick={toggleChat} aria-label="Close chat">✕</button>
            </div>
          </div>

          <div className="chat-messages" role="log" aria-live="polite" aria-label="Chat messages">
            {chatState.currentSession?.messages.length === 0 && (
              <div className="chat-welcome">
                <p>👋 Hi! I'm here to help you understand the documentation.</p>
                <p>Ask me anything about the content on this page!</p>
              </div>
            )}
            {chatState.currentSession?.messages.map(message => (
              <div key={message.id} className={`chat-message chat-message-${message.sender}`}>
                <div className="message-content">{message.content}</div>
                <div className="message-timestamp">{new Date(message.timestamp).toLocaleTimeString()}</div>
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-container">
            {chatState.isLoading && <div className="chat-loading"><span className="loading-spinner">⏳</span> Thinking...</div>}
            {chatState.error && <div className="chat-error">{chatState.error}</div>}

            {selectedText && (
              <div className="selected-text-container">
                <div className="selected-text-header">
                  <div className="selected-text-badge">
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                    <span>Selected Text</span>
                  </div>
                  <button
                    type="button"
                    className="selected-text-close"
                    onClick={() => setSelectedText(null)}
                    aria-label="Clear selected text"
                    title="Clear selection"
                  >
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M6 18L18 6M6 6l12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                  </button>
                </div>
                <div className="selected-text-content">
                  "{selectedText.length > 120 ? selectedText.substring(0, 120) + '...' : selectedText}"
                </div>
              </div>
            )}

            <form className="chat-input-form" onSubmit={handleSubmit}>
              <input
                ref={inputRef}
                type="text"
                name="message"
                className="chat-input"
                placeholder={selectedText ? "Ask about the selected text..." : "Ask a question..."}
                disabled={chatState.isLoading}
                autoComplete="off"
                aria-label="Chat message input"
                maxLength={1000}
              />
              <button type="submit" className="chat-send-button" disabled={chatState.isLoading} aria-label="Send message">
                Send
              </button>
            </form>
          </div>
        </div>
      )}
    </>
  );
};

export default BookChatbot;
