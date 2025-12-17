---
description: Generate a complete React chatbot widget with text selection, dual modes, and full styling
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Overview

Generate a production-ready React chatbot widget that includes:
1. Floating chat bubble with expand/collapse
2. Text selection detection for contextual queries
3. Two query modes: full search vs selected text only
4. Message display with source citations
5. Input area with send button and loading states
6. Complete API integration
7. Comprehensive CSS styling
8. Mode switching indicators and user feedback

## Implementation Steps

### 1. Analyze User Requirements

Extract from user input (if provided):
- Widget position (default: bottom-right)
- Primary color theme (default: #007bff)
- API endpoint URL
- Widget title (default: "AI Assistant")
- Max message history
- Custom icons or branding
- Animation preferences

### 2. Generate Main ChatWidget Component

```tsx
// components/ChatWidget/ChatWidget.tsx

import React, { useState, useEffect, useRef, useCallback } from 'react';
import './ChatWidget.css';

// ============================================================================
// TYPES & INTERFACES
// ============================================================================

interface Message {
  id: string;
  type: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  sources?: Source[];
  mode?: 'full' | 'selection';
}

interface Source {
  chunkId: string;
  content: string;
  similarityScore: number;
  metadata?: {
    filePath?: string;
    section?: string;
    chunkIndex?: number;
  };
}

interface ChatWidgetProps {
  apiEndpoint?: string;
  primaryColor?: string;
  position?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  title?: string;
  maxMessages?: number;
  placeholder?: string;
  enableSelection?: boolean;
}

// ============================================================================
// MAIN COMPONENT
// ============================================================================

const ChatWidget: React.FC<ChatWidgetProps> = ({
  apiEndpoint = '/api/chat/query',
  primaryColor = '#007bff',
  position = 'bottom-right',
  title = 'AI Assistant',
  maxMessages = 50,
  placeholder = 'Ask a question...',
  enableSelection = true,
}) => {
  // State management
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [queryMode, setQueryMode] = useState<'full' | 'selection'>('full');
  const [error, setError] = useState<string | null>(null);

  // Refs
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const selectionTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // ========================================================================
  // TEXT SELECTION DETECTION
  // ========================================================================

  const handleTextSelection = useCallback(() => {
    if (!enableSelection) return;

    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';

    // Clear existing timeout
    if (selectionTimeoutRef.current) {
      clearTimeout(selectionTimeoutRef.current);
    }

    // Debounce selection updates
    selectionTimeoutRef.current = setTimeout(() => {
      if (text.length > 0 && text.length <= 5000) {
        setSelectedText(text);
        setQueryMode('selection');

        // Add system message to inform user
        const systemMessage: Message = {
          id: `sys-${Date.now()}`,
          type: 'system',
          content: `Text selected (${text.length} characters). Questions will be answered based on this selection.`,
          timestamp: new Date(),
          mode: 'selection',
        };

        setMessages((prev) => [...prev, systemMessage]);

        // Auto-open widget if closed
        if (!isOpen) {
          setIsOpen(true);
        }
      } else if (text.length === 0 && queryMode === 'selection') {
        // Selection cleared
        setSelectedText('');
        setQueryMode('full');

        const systemMessage: Message = {
          id: `sys-${Date.now()}`,
          type: 'system',
          content: 'Selection cleared. Questions will now search the entire content.',
          timestamp: new Date(),
          mode: 'full',
        };

        setMessages((prev) => [...prev, systemMessage]);
      }
    }, 300);
  }, [enableSelection, isOpen, queryMode]);

  // Listen for text selection events
  useEffect(() => {
    if (enableSelection) {
      document.addEventListener('mouseup', handleTextSelection);
      document.addEventListener('keyup', handleTextSelection);

      return () => {
        document.removeEventListener('mouseup', handleTextSelection);
        document.removeEventListener('keyup', handleTextSelection);
        if (selectionTimeoutRef.current) {
          clearTimeout(selectionTimeoutRef.current);
        }
      };
    }
  }, [handleTextSelection, enableSelection]);

  // ========================================================================
  // MESSAGE HANDLING
  // ========================================================================

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      type: 'user',
      content: inputValue.trim(),
      timestamp: new Date(),
      mode: queryMode,
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Prepare request payload
      const requestBody: any = {
        question: inputValue.trim(),
      };

      // If in selection mode, add context
      if (queryMode === 'selection' && selectedText) {
        requestBody.context = selectedText;
        requestBody.mode = 'selection';
      }

      // Call API
      const response = await fetch(apiEndpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Create assistant message
      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        type: 'assistant',
        content: data.answer,
        timestamp: new Date(),
        sources: data.sources || [],
        mode: queryMode,
      };

      setMessages((prev) => {
        const newMessages = [...prev, assistantMessage];
        // Trim messages if exceeding max
        if (newMessages.length > maxMessages) {
          return newMessages.slice(-maxMessages);
        }
        return newMessages;
      });
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Unknown error occurred';
      setError(errorMessage);

      const errorMsg: Message = {
        id: `error-${Date.now()}`,
        type: 'system',
        content: `Error: ${errorMessage}`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleMode = () => {
    const newMode = queryMode === 'full' ? 'selection' : 'full';
    setQueryMode(newMode);

    const systemMessage: Message = {
      id: `sys-${Date.now()}`,
      type: 'system',
      content: newMode === 'full'
        ? 'Switched to full search mode'
        : 'Switched to selection mode (select text to use as context)',
      timestamp: new Date(),
      mode: newMode,
    };

    setMessages((prev) => [...prev, systemMessage]);
  };

  const clearChat = () => {
    setMessages([]);
    setError(null);
  };

  // ========================================================================
  // RENDER
  // ========================================================================

  return (
    <div
      className={`chat-widget chat-widget--${position}`}
      style={{ '--primary-color': primaryColor } as React.CSSProperties}
    >
      {/* Floating Chat Bubble */}
      {!isOpen && (
        <button
          className="chat-widget__bubble"
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
          {selectedText && (
            <span className="chat-widget__bubble-badge">
              <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor">
                <circle cx="6" cy="6" r="6" />
              </svg>
            </span>
          )}
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-widget__window">
          {/* Header */}
          <div className="chat-widget__header">
            <div className="chat-widget__header-title">
              <h3>{title}</h3>
              <div className="chat-widget__mode-indicator">
                <span className={`mode-badge mode-badge--${queryMode}`}>
                  {queryMode === 'full' ? '🌐 Full Search' : '📝 Selected Text'}
                </span>
              </div>
            </div>
            <div className="chat-widget__header-actions">
              {enableSelection && (
                <button
                  className="chat-widget__header-button"
                  onClick={toggleMode}
                  title={`Switch to ${queryMode === 'full' ? 'selection' : 'full'} mode`}
                  aria-label="Toggle search mode"
                >
                  <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                    <path d="M8 1.5A6.5 6.5 0 1 0 14.5 8A6.5 6.5 0 0 0 8 1.5zm0 11.5A5 5 0 1 1 13 8 5 5 0 0 1 8 13z"/>
                    <path d="M8 5.5a.5.5 0 0 1 .5.5v2h2a.5.5 0 0 1 0 1h-2v2a.5.5 0 0 1-1 0V9h-2a.5.5 0 0 1 0-1h2V6a.5.5 0 0 1 .5-.5z"/>
                  </svg>
                </button>
              )}
              <button
                className="chat-widget__header-button"
                onClick={clearChat}
                title="Clear chat"
                aria-label="Clear chat history"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                  <path d="M5.5 5.5A.5.5 0 0 1 6 6v6a.5.5 0 0 1-1 0V6a.5.5 0 0 1 .5-.5zm2.5 0a.5.5 0 0 1 .5.5v6a.5.5 0 0 1-1 0V6a.5.5 0 0 1 .5-.5zm3 .5a.5.5 0 0 0-1 0v6a.5.5 0 0 0 1 0V6z"/>
                  <path d="M14.5 3a1 1 0 0 1-1 1H13v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V4h-.5a1 1 0 0 1-1-1V2a1 1 0 0 1 1-1H6a1 1 0 0 1 1-1h2a1 1 0 0 1 1 1h3.5a1 1 0 0 1 1 1v1zM4.118 4 4 4.059V13a1 1 0 0 0 1 1h6a1 1 0 0 0 1-1V4.059L11.882 4H4.118zM2.5 3V2h11v1h-11z"/>
                </svg>
              </button>
              <button
                className="chat-widget__header-button"
                onClick={() => setIsOpen(false)}
                title="Close chat"
                aria-label="Close chat"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                  <path d="M4.646 4.646a.5.5 0 0 1 .708 0L8 7.293l2.646-2.647a.5.5 0 0 1 .708.708L8.707 8l2.647 2.646a.5.5 0 0 1-.708.708L8 8.707l-2.646 2.647a.5.5 0 0 1-.708-.708L7.293 8 4.646 5.354a.5.5 0 0 1 0-.708z"/>
                </svg>
              </button>
            </div>
          </div>

          {/* Selected Text Preview */}
          {queryMode === 'selection' && selectedText && (
            <div className="chat-widget__selection-preview">
              <div className="selection-preview__header">
                <span className="selection-preview__icon">📋</span>
                <span className="selection-preview__label">Selected Context</span>
                <button
                  className="selection-preview__clear"
                  onClick={() => {
                    setSelectedText('');
                    setQueryMode('full');
                  }}
                  aria-label="Clear selection"
                >
                  ×
                </button>
              </div>
              <div className="selection-preview__content">
                {selectedText.length > 150
                  ? `${selectedText.slice(0, 150)}...`
                  : selectedText}
              </div>
              <div className="selection-preview__meta">
                {selectedText.length} characters
              </div>
            </div>
          )}

          {/* Messages */}
          <div className="chat-widget__messages">
            {messages.length === 0 && (
              <div className="chat-widget__empty-state">
                <div className="empty-state__icon">💬</div>
                <h4>How can I help you?</h4>
                <p>
                  {enableSelection
                    ? 'Ask questions about the content or select text for contextual answers.'
                    : 'Ask me anything about the documentation.'}
                </p>
              </div>
            )}

            {messages.map((message) => (
              <MessageBubble key={message.id} message={message} />
            ))}

            {isLoading && (
              <div className="chat-widget__loading">
                <div className="loading-dots">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className="chat-widget__input-area">
            <textarea
              ref={inputRef}
              className="chat-widget__input"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={placeholder}
              rows={1}
              disabled={isLoading}
              aria-label="Message input"
            />
            <button
              className="chat-widget__send-button"
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 20 20"
                fill="currentColor"
              >
                <path d="M2.01 2.01c-.26-.26-.69-.15-.79.19L.04 8.53c-.1.34.18.66.52.66h6.94c.28 0 .5.22.5.5v.62c0 .28-.22.5-.5.5H.56c-.34 0-.62.32-.52.66l1.18 6.33c.1.34.53.45.79.19l16.97-16.97c.26-.26.15-.69-.19-.79L2.46.85c-.34-.1-.71.08-.45.34l-.01.82z"/>
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

// ============================================================================
// MESSAGE BUBBLE COMPONENT
// ============================================================================

interface MessageBubbleProps {
  message: Message;
}

const MessageBubble: React.FC<MessageBubbleProps> = ({ message }) => {
  const [showSources, setShowSources] = useState(false);

  const formatTimestamp = (date: Date) => {
    return new Intl.DateTimeFormat('en-US', {
      hour: 'numeric',
      minute: '2-digit',
      hour12: true,
    }).format(date);
  };

  return (
    <div className={`message message--${message.type}`}>
      <div className="message__content">
        {message.type === 'assistant' && (
          <div className="message__avatar">
            <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
              <circle cx="10" cy="10" r="8"/>
            </svg>
          </div>
        )}

        <div className="message__bubble">
          {message.mode && (
            <span className="message__mode-tag">
              {message.mode === 'selection' ? '📝' : '🌐'}
            </span>
          )}

          <div className="message__text">{message.content}</div>

          {message.sources && message.sources.length > 0 && (
            <div className="message__sources">
              <button
                className="message__sources-toggle"
                onClick={() => setShowSources(!showSources)}
              >
                {showSources ? '▼' : '▶'} {message.sources.length} source
                {message.sources.length > 1 ? 's' : ''}
              </button>

              {showSources && (
                <div className="message__sources-list">
                  {message.sources.map((source, idx) => (
                    <div key={source.chunkId} className="source-item">
                      <div className="source-item__header">
                        <span className="source-item__number">Source {idx + 1}</span>
                        <span className="source-item__score">
                          {(source.similarityScore * 100).toFixed(1)}% match
                        </span>
                      </div>
                      <div className="source-item__content">
                        {source.content.slice(0, 200)}
                        {source.content.length > 200 ? '...' : ''}
                      </div>
                      {source.metadata && (
                        <div className="source-item__metadata">
                          {source.metadata.filePath && (
                            <span className="source-item__file">
                              📄 {source.metadata.filePath}
                            </span>
                          )}
                          {source.metadata.section && (
                            <span className="source-item__section">
                              § {source.metadata.section}
                            </span>
                          )}
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}

          <div className="message__timestamp">
            {formatTimestamp(message.timestamp)}
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChatWidget;
```

### 3. Generate CSS Styling

```css
/* components/ChatWidget/ChatWidget.css */

:root {
  --chat-primary: var(--primary-color, #007bff);
  --chat-primary-dark: #0056b3;
  --chat-primary-light: #e7f3ff;
  --chat-bg: #ffffff;
  --chat-text: #333333;
  --chat-text-light: #666666;
  --chat-border: #e0e0e0;
  --chat-shadow: rgba(0, 0, 0, 0.1);
  --chat-shadow-lg: rgba(0, 0, 0, 0.15);
  --user-message-bg: var(--chat-primary);
  --user-message-text: #ffffff;
  --assistant-message-bg: #f5f5f5;
  --assistant-message-text: var(--chat-text);
  --system-message-bg: #fff3cd;
  --system-message-text: #856404;
  --error-bg: #f8d7da;
  --error-text: #721c24;
  --border-radius: 12px;
  --transition-speed: 0.3s;
}

/* ============================================================================
   MAIN WIDGET CONTAINER
   ============================================================================ */

.chat-widget {
  position: fixed;
  z-index: 9999;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
  font-size: 14px;
  line-height: 1.5;
}

.chat-widget--bottom-right {
  bottom: 20px;
  right: 20px;
}

.chat-widget--bottom-left {
  bottom: 20px;
  left: 20px;
}

.chat-widget--top-right {
  top: 20px;
  right: 20px;
}

.chat-widget--top-left {
  top: 20px;
  left: 20px;
}

/* ============================================================================
   FLOATING BUBBLE
   ============================================================================ */

.chat-widget__bubble {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: var(--chat-primary);
  color: white;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px var(--chat-shadow-lg);
  transition: all var(--transition-speed) ease;
  position: relative;
}

.chat-widget__bubble:hover {
  transform: scale(1.1);
  box-shadow: 0 6px 16px var(--chat-shadow-lg);
}

.chat-widget__bubble:active {
  transform: scale(0.95);
}

.chat-widget__bubble-badge {
  position: absolute;
  top: 4px;
  right: 4px;
  width: 16px;
  height: 16px;
  background: #28a745;
  border-radius: 50%;
  border: 2px solid white;
  display: flex;
  align-items: center;
  justify-content: center;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

/* ============================================================================
   CHAT WINDOW
   ============================================================================ */

.chat-widget__window {
  width: 380px;
  height: 600px;
  max-height: calc(100vh - 100px);
  background: var(--chat-bg);
  border-radius: var(--border-radius);
  box-shadow: 0 8px 24px var(--chat-shadow-lg);
  display: flex;
  flex-direction: column;
  overflow: hidden;
  animation: slideUp 0.3s ease;
}

@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Header */
.chat-widget__header {
  background: var(--chat-primary);
  color: white;
  padding: 16px;
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.chat-widget__header-title h3 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
}

.chat-widget__mode-indicator {
  margin-top: 4px;
}

.mode-badge {
  display: inline-block;
  font-size: 11px;
  padding: 2px 8px;
  border-radius: 12px;
  background: rgba(255, 255, 255, 0.2);
  font-weight: 500;
}

.mode-badge--selection {
  background: rgba(255, 235, 59, 0.3);
}

.chat-widget__header-actions {
  display: flex;
  gap: 8px;
}

.chat-widget__header-button {
  background: rgba(255, 255, 255, 0.2);
  border: none;
  color: white;
  width: 28px;
  height: 28px;
  border-radius: 6px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background var(--transition-speed);
}

.chat-widget__header-button:hover {
  background: rgba(255, 255, 255, 0.3);
}

/* Selection Preview */
.chat-widget__selection-preview {
  background: var(--chat-primary-light);
  border-bottom: 1px solid var(--chat-border);
  padding: 12px;
}

.selection-preview__header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 8px;
}

.selection-preview__icon {
  font-size: 16px;
}

.selection-preview__label {
  font-size: 12px;
  font-weight: 600;
  color: var(--chat-primary);
  flex: 1;
}

.selection-preview__clear {
  background: none;
  border: none;
  font-size: 20px;
  color: var(--chat-text-light);
  cursor: pointer;
  padding: 0;
  width: 20px;
  height: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.selection-preview__clear:hover {
  color: var(--chat-text);
}

.selection-preview__content {
  font-size: 12px;
  color: var(--chat-text);
  line-height: 1.4;
  padding: 8px;
  background: white;
  border-radius: 6px;
  max-height: 60px;
  overflow-y: auto;
}

.selection-preview__meta {
  font-size: 11px;
  color: var(--chat-text-light);
  margin-top: 4px;
}

/* ============================================================================
   MESSAGES AREA
   ============================================================================ */

.chat-widget__messages {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  background: #fafafa;
}

.chat-widget__messages::-webkit-scrollbar {
  width: 6px;
}

.chat-widget__messages::-webkit-scrollbar-track {
  background: transparent;
}

.chat-widget__messages::-webkit-scrollbar-thumb {
  background: var(--chat-border);
  border-radius: 3px;
}

.chat-widget__messages::-webkit-scrollbar-thumb:hover {
  background: var(--chat-text-light);
}

/* Empty State */
.chat-widget__empty-state {
  text-align: center;
  padding: 40px 20px;
  color: var(--chat-text-light);
}

.empty-state__icon {
  font-size: 48px;
  margin-bottom: 16px;
}

.chat-widget__empty-state h4 {
  margin: 0 0 8px 0;
  color: var(--chat-text);
  font-size: 16px;
}

.chat-widget__empty-state p {
  margin: 0;
  font-size: 13px;
  line-height: 1.5;
}

/* Loading Indicator */
.chat-widget__loading {
  display: flex;
  align-items: center;
  padding: 12px;
}

.loading-dots {
  display: flex;
  gap: 4px;
}

.loading-dots span {
  width: 8px;
  height: 8px;
  background: var(--chat-text-light);
  border-radius: 50%;
  animation: bounce 1.4s infinite ease-in-out both;
}

.loading-dots span:nth-child(1) {
  animation-delay: -0.32s;
}

.loading-dots span:nth-child(2) {
  animation-delay: -0.16s;
}

@keyframes bounce {
  0%, 80%, 100% {
    transform: scale(0);
  }
  40% {
    transform: scale(1);
  }
}

/* ============================================================================
   MESSAGE BUBBLES
   ============================================================================ */

.message {
  margin-bottom: 16px;
  animation: fadeIn 0.3s ease;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(10px); }
  to { opacity: 1; transform: translateY(0); }
}

.message__content {
  display: flex;
  gap: 8px;
  align-items: flex-start;
}

.message--user .message__content {
  flex-direction: row-reverse;
}

.message__avatar {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  background: var(--chat-primary);
  color: white;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.message__bubble {
  max-width: 75%;
  padding: 10px 14px;
  border-radius: var(--border-radius);
  position: relative;
}

.message--user .message__bubble {
  background: var(--user-message-bg);
  color: var(--user-message-text);
  border-bottom-right-radius: 4px;
}

.message--assistant .message__bubble {
  background: var(--assistant-message-bg);
  color: var(--assistant-message-text);
  border-bottom-left-radius: 4px;
}

.message--system .message__bubble {
  background: var(--system-message-bg);
  color: var(--system-message-text);
  font-size: 12px;
  max-width: 100%;
  text-align: center;
  border-radius: 6px;
}

.message__mode-tag {
  position: absolute;
  top: -8px;
  right: 8px;
  font-size: 12px;
  background: white;
  padding: 2px 6px;
  border-radius: 8px;
  box-shadow: 0 1px 3px var(--chat-shadow);
}

.message__text {
  margin: 0;
  word-wrap: break-word;
  white-space: pre-wrap;
}

.message__timestamp {
  font-size: 10px;
  color: rgba(0, 0, 0, 0.4);
  margin-top: 4px;
  text-align: right;
}

.message--user .message__timestamp {
  color: rgba(255, 255, 255, 0.7);
}

/* Sources */
.message__sources {
  margin-top: 12px;
  border-top: 1px solid var(--chat-border);
  padding-top: 8px;
}

.message__sources-toggle {
  background: none;
  border: none;
  color: var(--chat-primary);
  font-size: 12px;
  cursor: pointer;
  padding: 4px 0;
  font-weight: 500;
  display: flex;
  align-items: center;
  gap: 4px;
}

.message__sources-toggle:hover {
  text-decoration: underline;
}

.message__sources-list {
  margin-top: 8px;
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.source-item {
  background: white;
  border: 1px solid var(--chat-border);
  border-radius: 8px;
  padding: 10px;
  font-size: 12px;
}

.source-item__header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 6px;
}

.source-item__number {
  font-weight: 600;
  color: var(--chat-primary);
}

.source-item__score {
  font-size: 11px;
  color: var(--chat-text-light);
  background: var(--chat-primary-light);
  padding: 2px 6px;
  border-radius: 4px;
}

.source-item__content {
  color: var(--chat-text);
  line-height: 1.4;
  margin-bottom: 6px;
}

.source-item__metadata {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  font-size: 11px;
  color: var(--chat-text-light);
}

.source-item__file,
.source-item__section {
  display: inline-flex;
  align-items: center;
  gap: 4px;
}

/* ============================================================================
   INPUT AREA
   ============================================================================ */

.chat-widget__input-area {
  padding: 16px;
  background: var(--chat-bg);
  border-top: 1px solid var(--chat-border);
  display: flex;
  gap: 8px;
  align-items: flex-end;
}

.chat-widget__input {
  flex: 1;
  border: 1px solid var(--chat-border);
  border-radius: 20px;
  padding: 10px 16px;
  font-family: inherit;
  font-size: 14px;
  resize: none;
  max-height: 120px;
  transition: border-color var(--transition-speed);
}

.chat-widget__input:focus {
  outline: none;
  border-color: var(--chat-primary);
}

.chat-widget__input:disabled {
  background: #f5f5f5;
  cursor: not-allowed;
}

.chat-widget__send-button {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background: var(--chat-primary);
  color: white;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  transition: all var(--transition-speed);
}

.chat-widget__send-button:hover:not(:disabled) {
  background: var(--chat-primary-dark);
  transform: scale(1.05);
}

.chat-widget__send-button:active:not(:disabled) {
  transform: scale(0.95);
}

.chat-widget__send-button:disabled {
  background: var(--chat-border);
  cursor: not-allowed;
  opacity: 0.5;
}

/* ============================================================================
   RESPONSIVE
   ============================================================================ */

@media (max-width: 480px) {
  .chat-widget__window {
    width: calc(100vw - 40px);
    height: calc(100vh - 100px);
  }

  .chat-widget--bottom-right,
  .chat-widget--bottom-left {
    bottom: 10px;
    right: 10px;
    left: 10px;
  }
}
```

### 4. Generate API Integration Hook

```typescript
// hooks/useChatAPI.ts

import { useState, useCallback } from 'react';

interface ChatAPIConfig {
  endpoint: string;
  headers?: Record<string, string>;
}

interface QueryRequest {
  question: string;
  context?: string;
  mode?: 'full' | 'selection';
  topK?: number;
  similarityThreshold?: number;
}

export const useChatAPI = (config: ChatAPIConfig) => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const query = useCallback(
    async (request: QueryRequest) => {
      setIsLoading(true);
      setError(null);

      try {
        const response = await fetch(config.endpoint, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...config.headers,
          },
          body: JSON.stringify(request),
        });

        if (!response.ok) {
          throw new Error(`API error: ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (err) {
        const error = err instanceof Error ? err : new Error('Unknown error');
        setError(error);
        throw error;
      } finally {
        setIsLoading(false);
      }
    },
    [config]
  );

  return { query, isLoading, error };
};
```

### 5. Generate Usage Example

```tsx
// App.tsx or wherever you want to use the widget

import React from 'react';
import ChatWidget from './components/ChatWidget/ChatWidget';

function App() {
  return (
    <div className="app">
      {/* Your main content */}
      <main>
        <h1>My Documentation</h1>
        <p>Select any text to ask questions about it!</p>
      </main>

      {/* Chat Widget */}
      <ChatWidget
        apiEndpoint="/api/chat/query"
        primaryColor="#007bff"
        position="bottom-right"
        title="AI Assistant"
        maxMessages={50}
        placeholder="Ask a question..."
        enableSelection={true}
      />
    </div>
  );
}

export default App;
```

### 6. Generate TypeScript Declarations

```typescript
// types/chat.d.ts

export interface Message {
  id: string;
  type: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  sources?: Source[];
  mode?: 'full' | 'selection';
}

export interface Source {
  chunkId: string;
  content: string;
  similarityScore: number;
  metadata?: SourceMetadata;
}

export interface SourceMetadata {
  filePath?: string;
  section?: string;
  chunkIndex?: number;
  [key: string]: any;
}

export interface ChatWidgetProps {
  apiEndpoint?: string;
  primaryColor?: string;
  position?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  title?: string;
  maxMessages?: number;
  placeholder?: string;
  enableSelection?: boolean;
}
```

### 7. Generate Test Suite

```typescript
// components/ChatWidget/ChatWidget.test.tsx

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import ChatWidget from './ChatWidget';

describe('ChatWidget', () => {
  it('renders floating bubble when closed', () => {
    render(<ChatWidget />);
    const bubble = screen.getByLabelText('Open chat');
    expect(bubble).toBeInTheDocument();
  });

  it('opens widget when bubble is clicked', () => {
    render(<ChatWidget />);
    const bubble = screen.getByLabelText('Open chat');
    fireEvent.click(bubble);
    expect(screen.getByText('AI Assistant')).toBeInTheDocument();
  });

  it('detects text selection', async () => {
    render(<ChatWidget enableSelection={true} />);

    // Simulate text selection
    const selection = 'Test selected text';
    window.getSelection = jest.fn(() => ({
      toString: () => selection,
    })) as any;

    fireEvent.mouseUp(document);

    await waitFor(() => {
      expect(screen.getByText(/Text selected/)).toBeInTheDocument();
    });
  });

  it('sends message on button click', async () => {
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ answer: 'Test answer', sources: [] }),
      })
    ) as jest.Mock;

    render(<ChatWidget />);
    fireEvent.click(screen.getByLabelText('Open chat'));

    const input = screen.getByPlaceholderText('Ask a question...');
    await userEvent.type(input, 'Test question');

    const sendButton = screen.getByLabelText('Send message');
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(screen.getByText('Test answer')).toBeInTheDocument();
    });
  });

  it('toggles between full and selection mode', () => {
    render(<ChatWidget enableSelection={true} />);
    fireEvent.click(screen.getByLabelText('Open chat'));

    const toggleButton = screen.getByLabelText('Toggle search mode');
    fireEvent.click(toggleButton);

    expect(screen.getByText(/Switched to selection mode/)).toBeInTheDocument();
  });
});
```

## Deliverables

When complete, provide:

1. **ChatWidget.tsx** - Main component with all features
2. **ChatWidget.css** - Complete styling with animations
3. **useChatAPI.ts** - API integration hook
4. **types/chat.d.ts** - TypeScript definitions
5. **ChatWidget.test.tsx** - Comprehensive tests
6. **Usage example** - Integration guide
7. **README** - Component documentation

## Features Included

- ✅ Floating chat bubble with badge indicator
- ✅ Text selection detection with debouncing
- ✅ Dual query modes (full search / selected text)
- ✅ Message display with timestamps
- ✅ Source citations with expandable details
- ✅ Loading states and error handling
- ✅ Input area with auto-resize
- ✅ Mode switching with visual indicators
- ✅ Selected text preview panel
- ✅ Clear chat functionality
- ✅ Empty state messaging
- ✅ Smooth animations and transitions
- ✅ Responsive design
- ✅ Accessibility features (ARIA labels)
- ✅ Customizable theming
- ✅ Position flexibility
- ✅ Message history management

## Customization Options

The widget supports:
- Custom API endpoint
- Primary color theming
- Position (4 corners)
- Custom title and placeholder
- Message limit configuration
- Enable/disable text selection
- Custom headers for API calls
