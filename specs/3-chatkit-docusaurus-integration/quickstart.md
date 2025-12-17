# Quickstart: ChatKit Integration with Docusaurus Book Site

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Docusaurus project already set up
- Access to backend chat APIs at `/api/v1/chat` and `/api/v1/chat/selected-text`

## Setup

### 1. Install Dependencies
```bash
cd docusaurus-project
npm install @openai/chatkit  # or whatever chat library is used
```

### 2. Create Chat Component
Create the reusable chatbot component:

```bash
mkdir -p src/components
touch src/components/BookChatbot.tsx
```

### 3. Override Docusaurus Layout
Create the layout override to inject chatbot globally:

```bash
mkdir -p src/theme/Layout
touch src/theme/Layout/index.tsx
```

## Development Workflow

### 1. Start Development Server
```bash
cd docusaurus-project
npm run start
```

### 2. Component Structure
The main files to work with:
- `src/components/BookChatbot.tsx` - The reusable chat component
- `src/theme/Layout/index.tsx` - Layout wrapper that injects the chatbot

### 3. Testing
```bash
# Run component tests
npm run test

# Run end-to-end tests
npm run test:e2e
```

## Key Integration Points

### Backend API Endpoints
- `/api/v1/chat` - For general chat queries
- `/api/v1/chat/selected-text` - For queries about selected text

### Global Injection
The chatbot is injected once in the Layout component and appears on all pages.

## Common Tasks

### Add Chat to a Page
No action needed - chat appears automatically on all documentation pages.

### Customize Chat Styling
Modify styles in `src/components/BookChatbot.tsx` or add CSS classes.

### Handle Selected Text
The component captures selected text and sends it to the `/api/v1/chat/selected-text` endpoint.

## Environment Variables
```bash
# Backend API base URL
REACT_APP_CHAT_API_BASE_URL=https://your-backend-api.com
```

## Troubleshooting

### Chat Not Appearing
- Verify the Layout override is properly implemented
- Check that the BookChatbot component is being rendered

### API Connection Issues
- Verify backend endpoints are accessible
- Check network requests in browser dev tools
- Confirm CORS settings allow requests from your domain