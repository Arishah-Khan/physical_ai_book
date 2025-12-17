# 📘 Frontend Workflow: ChatKit Integration with Docusaurus Book Site

> *Scope:* This document describes *only the frontend-side workflow. The backend is already implemented and exposes stable chat endpoints. This workflow focuses on how **ChatKit is configured with backend endpoints* and *embedded across all Docusaurus book pages*.

---

## 1. Objective

* Integrate *ChatKit (React UI)* with existing backend chat APIs
* Embed the chatbot on *every documentation page* in docusaurus-project/docs
* Ensure the chatbot works consistently without modifying backend logic

---

## 2. Existing Repository Context

This workflow aligns with the following structure:


docusaurus-project/
 ├─ .docusaurus/
 ├─ blog/
 ├─ docs/
 ├─ src/
 │   ├─ components/
 │   └─ theme/
 ├─ static/
 └─ node_modules/


The backend exists separately and is accessed via HTTP endpoints.

---

## 3. Frontend Architecture Overview


User
 ↓
Docusaurus Docs Page
 ↓
ChatKit React Component
 ↓
Backend Chat API (existing)
 ↓
ChatKit UI renders response


---

## 4. ChatKit’s Role (Frontend Only)

ChatKit is responsible for:

* Rendering the chat interface
* Managing chat state (messages, session/thread id)
* Sending user input to backend endpoints
* Displaying backend responses inside the UI

ChatKit *does not*:

* perform retrieval
* call Qdrant directly
* execute any agent logic

---

## 5. Backend Endpoint Usage (Read-Only)

The frontend treats backend APIs as *black-box services*.

### Endpoints consumed by ChatKit:


POST /api/v1/chat
POST /api/v1/chat/selected-text


ChatKit only:

* sends request payloads
* receives JSON responses

No backend modification is required.

---

## 6. ChatKit Configuration Workflow

### Step 1: Install ChatKit in docusaurus-project


npm install @openai/chatkit


---

### Step 2: Create a Reusable Chatbot Component

*Location:*


docusaurus-project/src/components/BookChatbot.tsx


Responsibilities:

* Initialize ChatKit
* Configure backend API URL
* Attach session/thread identifier

The component acts as a *single integration point* between ChatKit and backend APIs.

---

## 7. Global Chatbot Injection (All Pages)

### Step 3: Override Docusaurus Layout

*Location:*


docusaurus-project/src/theme/Layout/index.ts


Workflow:

* Wrap the default Docusaurus layout
* Render <BookChatbot /> once
* Make the chatbot visible on *every documentation page*

This avoids repeating chatbot code on individual pages.

---

## 8. Page-Level Awareness (Optional)

Although the backend is fixed, the frontend can still send page context:

* Current route path
* Page identifier

This information can be included in ChatKit requests if needed for analytics or future enhancements.

---

## 9. Selected Text Interaction Flow


User selects text on a docs page
 → Frontend captures selected text
 → ChatKit sends request to:
   POST /api/v1/chat/selected-text
 → Backend returns constrained answer
 → ChatKit renders response


The selection logic lives entirely in the frontend.

---

## 10. End-to-End Frontend Interaction Flow

### Normal Question


User → ChatKit UI
     → Backend /api/v1/chat
     → ChatKit UI → User


### Selected Text Question


User selects text
 → ChatKit UI
 → Backend /api/v1/chat/selected-text
 → ChatKit UI


---

## 11. Design Principles (Frontend Perspective)

* Backend treated as stable service
* Single ChatKit integration point
* Global injection via layout override
* No duplication across pages
* Clean separation between content and interaction

---

## 12. SpecKit-Ready Summary

> This workflow integrates a ChatKit-based React chatbot into a Docusaurus documentation site by configuring it to consume existing backend chat APIs. The chatbot is injected globally using a layout override, ensuring availability across all book pages without altering backend logic. The frontend handles user interaction, selected-text capture, and response rendering, while the backend remains a black-box conversational service.

---

## 13. Keywords (Frontend Spec)

* ChatKit UI Integration
* Docusaurus Global Chatbot
* Frontend-Only Workflow
* Backend-as-a-Service Chatbot