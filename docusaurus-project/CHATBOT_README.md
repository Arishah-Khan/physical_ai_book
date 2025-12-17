# ChatKit Integration - Documentation Assistant

## Overview
This project integrates a chatbot component into the Docusaurus documentation site, providing users with an AI-powered assistant to help them understand the documentation content.

## Features

### Phase 1: Chat UI (User Story 1)
- Floating chat button in bottom-right corner
- Expandable chat window with message history
- Responsive design that works on mobile and desktop
- Toggle show/hide functionality
- Message input with send button
- Styled to match Docusaurus theme

### Phase 2: Backend Integration (User Story 2)
- Integration with `/api/v1/chat` endpoint for general queries
- Integration with `/api/v1/chat/selected-text` endpoint for context-aware queries
- Selected text capture functionality
- Page context sent with each query (URL, title, path)
- Loading states during API requests
- Error handling and user feedback

### Phase 3: Session Persistence (User Story 3)
- Chat session persists across page navigations
- Messages stored in localStorage
- Session timeout after 30 minutes of inactivity
- Clear history button to start fresh session
- Session restoration on page refresh

### Phase 4: Accessibility & Polish
- Keyboard navigation support (ESC to close)
- ARIA labels and roles for screen readers
- Focus management
- Input validation (max 1000 characters)
- Responsive design for mobile devices

## File Structure

```
docusaurus-project/
├── src/
│   ├── components/
│   │   ├── BookChatbot.tsx         # Main chatbot component
│   │   └── BookChatbot.css         # Chatbot styles
│   ├── services/
│   │   └── chatApi.ts              # API communication service
│   ├── types/
│   │   └── chat.ts                 # TypeScript interfaces
│   ├── utils/
│   │   └── sessionStorage.ts       # Session persistence utilities
│   └── theme/
│       └── Layout/
│           └── index.tsx           # Docusaurus layout override
```

## Configuration

### Environment Variables
Set the backend API URL in your environment:

```bash
REACT_APP_CHAT_API_BASE_URL=https://your-backend-api.com
```

If not set, defaults to `http://localhost:8000`

## Development

### Start Development Server
```bash
cd docusaurus-project
npm run start
```

### Build for Production
```bash
npm run build
```

### Serve Production Build
```bash
npm run serve
```

## API Endpoints

### POST /api/v1/chat
General chat queries with optional page context and session ID.

**Request:**
```json
{
  "message": "string",
  "sessionId": "string (optional)",
  "pageContext": {
    "url": "string",
    "title": "string",
    "path": "string"
  }
}
```

**Response:**
```json
{
  "message": "string",
  "sources": ["string"],
  "sessionId": "string",
  "timestamp": "ISO 8601 date"
}
```

### POST /api/v1/chat/selected-text
Queries about selected text from documentation pages.

**Request:**
```json
{
  "message": "string",
  "selectedText": "string",
  "sessionId": "string (optional)",
  "pageContext": {
    "url": "string",
    "title": "string",
    "path": "string"
  }
}
```

**Response:** Same as `/api/v1/chat`

## Usage

### For Users
1. Click the chat button (💬) in the bottom-right corner
2. Type your question about the documentation
3. Optionally select text on the page before asking for context-aware help
4. Navigate between pages - your chat history persists
5. Use the trash icon (🗑️) to clear chat history
6. Press ESC to close the chat window

### For Developers
The chatbot component is automatically injected on all pages via the Docusaurus Layout override. No additional configuration is needed.

## Accessibility

- Full keyboard navigation support
- ARIA labels and roles for screen readers
- Focus management for improved UX
- High contrast support via Docusaurus theme variables
- Responsive design for various screen sizes

## Performance

- Chat UI is lazy-loaded when toggled
- Session data stored in localStorage for fast restoration
- Optimized re-renders using React hooks
- Build size impact: < 0.5 seconds page load increase

## Browser Support

- Chrome/Edge (latest)
- Firefox (latest)
- Safari (latest)
- Mobile browsers (iOS Safari, Chrome Mobile)

## Troubleshooting

### Chat not appearing
- Verify the Layout override is in place at `src/theme/Layout/index.tsx`
- Check browser console for errors
- Ensure Docusaurus build completed successfully

### API connection issues
- Check that the backend is running and accessible
- Verify CORS settings allow requests from your domain
- Check environment variable `REACT_APP_CHAT_API_BASE_URL`
- Inspect network requests in browser DevTools

### Session not persisting
- Check that localStorage is enabled in browser
- Verify no browser extensions are blocking storage
- Check for storage quota exceeded errors in console

## Future Enhancements

Potential improvements for future iterations:
- Markdown rendering in chat messages
- Code syntax highlighting in responses
- Multi-language support
- Analytics integration
- Rate limiting feedback
- Typing indicators
- Message reactions
- Export chat history

## License

This implementation follows the project's license terms.
