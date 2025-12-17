# Data Model: ChatKit Integration with Docusaurus Book Site

## Overview
This document describes the data models relevant to the ChatKit integration with the Docusaurus book site. Since this is primarily a frontend integration with existing backend APIs, the data models focus on the client-side state and data flow.

## Client-Side Data Models

### Chat Session
**Description**: Represents a conversation between user and the chatbot, including message history and session identifier

**Fields**:
- `id`: string - Unique identifier for the chat session
- `messages`: Array<Message> - List of messages in the conversation
- `createdAt`: Date - When the session was created
- `lastActiveAt`: Date - When the last message was sent/received

### Message
**Description**: Represents a single message in the chat conversation

**Fields**:
- `id`: string - Unique identifier for the message
- `content`: string - The text content of the message
- `sender`: 'user' | 'bot' - Indicates who sent the message
- `timestamp`: Date - When the message was sent/received
- `status`: 'sending' | 'sent' | 'delivered' | 'error' - Current status of the message

### User Query
**Description**: The text input from the user to the chatbot, potentially including selected text from the documentation page

**Fields**:
- `text`: string - The main query text
- `selectedText`: string | null - Text selected on the documentation page (if applicable)
- `pageContext`: PageContext | null - Information about the current documentation page
- `timestamp`: Date - When the query was submitted

### Backend Response
**Description**: The structured response from the backend API containing the chatbot's answer to the user's query

**Fields**:
- `message`: string - The chatbot's response text
- `sources`: Array<string> - List of source documents/references used in the response
- `timestamp`: Date - When the response was generated

### Page Context
**Description**: Information about the current documentation page that may be included in requests for analytics or enhanced responses

**Fields**:
- `url`: string - The URL of the current page
- `title`: string - The title of the current page
- `path`: string - The relative path of the current page
- `section`: string - The section of the documentation

## State Management

### Chat State
**Description**: Client-side state for managing the current chat session and UI state

**Fields**:
- `currentSession`: ChatSession | null - The active chat session
- `isLoading`: boolean - Whether the chat is currently loading a response
- `error`: string | null - Any error messages
- `isVisible`: boolean - Whether the chat UI is currently visible

## Validation Rules

### Message Validation
- Content must not be empty (min length: 1 character)
- Content must not exceed 1000 characters (max length)
- Timestamp must be within reasonable range

### User Query Validation
- Query text must not be empty
- Selected text (if present) should be less than 500 characters
- Page context should be properly formatted with required fields

## State Transitions

### Chat Session States
1. `idle` - No active session
2. `active` - Session exists and user can send messages
3. `error` - Session encountered an error
4. `terminated` - Session has ended

### Message States
1. `sending` - Message is being sent to backend
2. `sent` - Message sent successfully to backend
3. `delivered` - Response received from backend
4. `error` - Error occurred during sending or receiving