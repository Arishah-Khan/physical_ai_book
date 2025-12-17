/**
 * Session Storage Utilities
 * Handles persistence of chat sessions in browser storage
 */

import type { ChatSession } from '../types/chat';

const STORAGE_KEY = 'docusaurus_chat_session';
const SESSION_TIMEOUT_MS = 30 * 60 * 1000; // 30 minutes

/**
 * Save chat session to localStorage
 */
export function saveSession(session: ChatSession): void {
  try {
    const sessionData = {
      session,
      savedAt: new Date().toISOString(),
    };
    localStorage.setItem(STORAGE_KEY, JSON.stringify(sessionData));
  } catch (error) {
    console.error('Error saving session:', error);
  }
}

/**
 * Load chat session from localStorage
 * Returns null if no session exists or session has expired
 */
export function loadSession(): ChatSession | null {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      return null;
    }

    const sessionData = JSON.parse(stored);
    const savedAt = new Date(sessionData.savedAt);
    const now = new Date();

    // Check if session has expired
    if (now.getTime() - savedAt.getTime() > SESSION_TIMEOUT_MS) {
      clearSession();
      return null;
    }

    // Parse dates in the session
    const session: ChatSession = {
      ...sessionData.session,
      createdAt: new Date(sessionData.session.createdAt),
      lastActiveAt: new Date(sessionData.session.lastActiveAt),
      messages: sessionData.session.messages.map((msg: any) => ({
        ...msg,
        timestamp: new Date(msg.timestamp),
      })),
    };

    return session;
  } catch (error) {
    console.error('Error loading session:', error);
    return null;
  }
}

/**
 * Clear chat session from localStorage
 */
export function clearSession(): void {
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch (error) {
    console.error('Error clearing session:', error);
  }
}

/**
 * Check if a session is still valid (not expired)
 */
export function isSessionValid(session: ChatSession): boolean {
  const now = new Date();
  const lastActive = new Date(session.lastActiveAt);
  return now.getTime() - lastActive.getTime() < SESSION_TIMEOUT_MS;
}
