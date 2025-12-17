---
description: Generate a production-ready React hook for REST API integration with retry logic, error handling, and state management
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Overview

Generate a comprehensive React custom hook for API integration that includes:
1. Support for all HTTP methods (GET, POST, PUT, DELETE, PATCH)
2. Loading, error, and data state management
3. Automatic retry logic with exponential backoff
4. Request cancellation on unmount
5. Error handling with custom error types
6. Response caching (optional)
7. Request/response interceptors
8. TypeScript support with generics
9. Abort controller for cleanup
10. Configuration options

## Implementation Steps

### 1. Analyze User Requirements

Extract from user input (if provided):
- Base API URL
- Default headers
- Retry configuration (attempts, delay)
- Cache settings
- Timeout duration
- Error handling strategy
- Authentication method

### 2. Generate Core Types and Interfaces

```typescript
// hooks/useAPI/types.ts

export type HTTPMethod = 'GET' | 'POST' | 'PUT' | 'DELETE' | 'PATCH';

export interface APIConfig {
  baseURL?: string;
  headers?: Record<string, string>;
  timeout?: number;
  retryAttempts?: number;
  retryDelay?: number;
  enableCache?: boolean;
  cacheDuration?: number; // in milliseconds
}

export interface RequestOptions<TBody = any> {
  method?: HTTPMethod;
  headers?: Record<string, string>;
  body?: TBody;
  params?: Record<string, string | number | boolean>;
  timeout?: number;
  retry?: boolean;
  retryAttempts?: number;
  retryDelay?: number;
  cache?: boolean;
  onUploadProgress?: (progress: number) => void;
  signal?: AbortSignal;
}

export interface APIResponse<TData = any> {
  data: TData;
  status: number;
  statusText: string;
  headers: Headers;
}

export interface APIError {
  message: string;
  status?: number;
  statusText?: string;
  code?: string;
  details?: any;
  timestamp: Date;
}

export interface APIState<TData = any> {
  data: TData | null;
  loading: boolean;
  error: APIError | null;
}

export interface APIHookResult<TData = any> {
  data: TData | null;
  loading: boolean;
  error: APIError | null;
  refetch: () => Promise<void>;
  reset: () => void;
}

export interface MutationResult<TData = any, TVariables = any> {
  mutate: (variables?: TVariables) => Promise<TData>;
  data: TData | null;
  loading: boolean;
  error: APIError | null;
  reset: () => void;
}

// Interceptor types
export type RequestInterceptor = (config: RequestOptions) => RequestOptions | Promise<RequestOptions>;
export type ResponseInterceptor = <T>(response: APIResponse<T>) => APIResponse<T> | Promise<APIResponse<T>>;
export type ErrorInterceptor = (error: APIError) => APIError | Promise<APIError>;
```

### 3. Generate API Client Class

```typescript
// hooks/useAPI/apiClient.ts

import type {
  APIConfig,
  RequestOptions,
  APIResponse,
  APIError,
  RequestInterceptor,
  ResponseInterceptor,
  ErrorInterceptor,
} from './types';

class APIClient {
  private config: APIConfig;
  private cache: Map<string, { data: any; timestamp: number }>;
  private requestInterceptors: RequestInterceptor[] = [];
  private responseInterceptors: ResponseInterceptor[] = [];
  private errorInterceptors: ErrorInterceptor[] = [];

  constructor(config: APIConfig = {}) {
    this.config = {
      baseURL: '',
      headers: {
        'Content-Type': 'application/json',
      },
      timeout: 30000,
      retryAttempts: 3,
      retryDelay: 1000,
      enableCache: false,
      cacheDuration: 5 * 60 * 1000, // 5 minutes
      ...config,
    };
    this.cache = new Map();
  }

  // Interceptor management
  addRequestInterceptor(interceptor: RequestInterceptor): void {
    this.requestInterceptors.push(interceptor);
  }

  addResponseInterceptor(interceptor: ResponseInterceptor): void {
    this.responseInterceptors.push(interceptor);
  }

  addErrorInterceptor(interceptor: ErrorInterceptor): void {
    this.errorInterceptors.push(interceptor);
  }

  // Build full URL with query parameters
  private buildURL(endpoint: string, params?: Record<string, any>): string {
    const baseURL = this.config.baseURL || '';
    const url = new URL(endpoint, baseURL);

    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined && value !== null) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    return url.toString();
  }

  // Generate cache key
  private getCacheKey(url: string, options: RequestOptions): string {
    return `${options.method || 'GET'}:${url}:${JSON.stringify(options.body || {})}`;
  }

  // Check cache
  private getFromCache<T>(key: string): T | null {
    if (!this.config.enableCache) return null;

    const cached = this.cache.get(key);
    if (!cached) return null;

    const now = Date.now();
    const isExpired = now - cached.timestamp > (this.config.cacheDuration || 0);

    if (isExpired) {
      this.cache.delete(key);
      return null;
    }

    return cached.data as T;
  }

  // Save to cache
  private saveToCache(key: string, data: any): void {
    if (!this.config.enableCache) return;

    this.cache.set(key, {
      data,
      timestamp: Date.now(),
    });
  }

  // Clear cache
  clearCache(pattern?: string): void {
    if (pattern) {
      Array.from(this.cache.keys()).forEach(key => {
        if (key.includes(pattern)) {
          this.cache.delete(key);
        }
      });
    } else {
      this.cache.clear();
    }
  }

  // Apply request interceptors
  private async applyRequestInterceptors(
    options: RequestOptions
  ): Promise<RequestOptions> {
    let config = { ...options };

    for (const interceptor of this.requestInterceptors) {
      config = await interceptor(config);
    }

    return config;
  }

  // Apply response interceptors
  private async applyResponseInterceptors<T>(
    response: APIResponse<T>
  ): Promise<APIResponse<T>> {
    let result = response;

    for (const interceptor of this.responseInterceptors) {
      result = await interceptor(result);
    }

    return result;
  }

  // Apply error interceptors
  private async applyErrorInterceptors(error: APIError): Promise<APIError> {
    let result = error;

    for (const interceptor of this.errorInterceptors) {
      result = await interceptor(result);
    }

    return result;
  }

  // Create error object
  private createError(
    message: string,
    status?: number,
    statusText?: string,
    details?: any
  ): APIError {
    return {
      message,
      status,
      statusText,
      details,
      timestamp: new Date(),
    };
  }

  // Sleep for retry delay
  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  // Make HTTP request with retry logic
  async request<TData = any, TBody = any>(
    endpoint: string,
    options: RequestOptions<TBody> = {}
  ): Promise<APIResponse<TData>> {
    // Apply request interceptors
    const processedOptions = await this.applyRequestInterceptors(options);

    const method = processedOptions.method || 'GET';
    const url = this.buildURL(endpoint, processedOptions.params);
    const cacheKey = this.getCacheKey(url, processedOptions);

    // Check cache for GET requests
    if (method === 'GET' && (processedOptions.cache ?? this.config.enableCache)) {
      const cachedData = this.getFromCache<TData>(cacheKey);
      if (cachedData) {
        return {
          data: cachedData,
          status: 200,
          statusText: 'OK (Cached)',
          headers: new Headers(),
        };
      }
    }

    const maxRetries = processedOptions.retryAttempts ?? this.config.retryAttempts ?? 3;
    const retryDelay = processedOptions.retryDelay ?? this.config.retryDelay ?? 1000;
    const timeout = processedOptions.timeout ?? this.config.timeout ?? 30000;

    let lastError: APIError | null = null;
    let attempt = 0;

    while (attempt <= maxRetries) {
      try {
        // Create abort controller for timeout
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), timeout);

        // Merge signals if user provided one
        const signal = processedOptions.signal
          ? this.mergeAbortSignals([controller.signal, processedOptions.signal])
          : controller.signal;

        // Prepare request
        const headers = new Headers({
          ...this.config.headers,
          ...processedOptions.headers,
        });

        const requestInit: RequestInit = {
          method,
          headers,
          signal,
        };

        // Add body for non-GET requests
        if (processedOptions.body && method !== 'GET') {
          if (headers.get('Content-Type')?.includes('application/json')) {
            requestInit.body = JSON.stringify(processedOptions.body);
          } else {
            requestInit.body = processedOptions.body as any;
          }
        }

        // Make request
        const response = await fetch(url, requestInit);
        clearTimeout(timeoutId);

        // Handle non-OK responses
        if (!response.ok) {
          let errorDetails: any;
          try {
            errorDetails = await response.json();
          } catch {
            errorDetails = await response.text();
          }

          const error = this.createError(
            `Request failed with status ${response.status}`,
            response.status,
            response.statusText,
            errorDetails
          );

          throw await this.applyErrorInterceptors(error);
        }

        // Parse response
        let data: TData;
        const contentType = response.headers.get('Content-Type');

        if (contentType?.includes('application/json')) {
          data = await response.json();
        } else if (contentType?.includes('text/')) {
          data = (await response.text()) as any;
        } else {
          data = (await response.blob()) as any;
        }

        const apiResponse: APIResponse<TData> = {
          data,
          status: response.status,
          statusText: response.statusText,
          headers: response.headers,
        };

        // Apply response interceptors
        const processedResponse = await this.applyResponseInterceptors(apiResponse);

        // Cache GET requests
        if (method === 'GET' && (processedOptions.cache ?? this.config.enableCache)) {
          this.saveToCache(cacheKey, processedResponse.data);
        }

        return processedResponse;

      } catch (error) {
        // Handle AbortError
        if (error instanceof Error && error.name === 'AbortError') {
          const abortError = this.createError(
            'Request timeout or cancelled',
            0,
            'Aborted'
          );
          throw await this.applyErrorInterceptors(abortError);
        }

        // Handle network errors
        if (error instanceof TypeError) {
          lastError = this.createError(
            'Network error - please check your connection',
            0,
            'NetworkError',
            error.message
          );
        } else if ((error as APIError).status !== undefined) {
          lastError = error as APIError;
        } else {
          lastError = this.createError(
            error instanceof Error ? error.message : 'Unknown error occurred',
            0,
            'UnknownError',
            error
          );
        }

        // Don't retry on client errors (4xx) or if retry is disabled
        const shouldRetry = processedOptions.retry !== false &&
          (!lastError.status || lastError.status >= 500) &&
          attempt < maxRetries;

        if (!shouldRetry) {
          throw await this.applyErrorInterceptors(lastError);
        }

        // Exponential backoff
        const delay = retryDelay * Math.pow(2, attempt);
        await this.sleep(delay);
        attempt++;
      }
    }

    // Should never reach here, but TypeScript needs it
    throw await this.applyErrorInterceptors(
      lastError || this.createError('Max retries exceeded')
    );
  }

  // Merge multiple abort signals
  private mergeAbortSignals(signals: AbortSignal[]): AbortSignal {
    const controller = new AbortController();

    for (const signal of signals) {
      if (signal.aborted) {
        controller.abort();
        break;
      }
      signal.addEventListener('abort', () => controller.abort());
    }

    return controller.signal;
  }

  // Convenience methods
  async get<TData = any>(
    endpoint: string,
    options?: Omit<RequestOptions, 'method' | 'body'>
  ): Promise<APIResponse<TData>> {
    return this.request<TData>(endpoint, { ...options, method: 'GET' });
  }

  async post<TData = any, TBody = any>(
    endpoint: string,
    body?: TBody,
    options?: Omit<RequestOptions<TBody>, 'method' | 'body'>
  ): Promise<APIResponse<TData>> {
    return this.request<TData, TBody>(endpoint, { ...options, method: 'POST', body });
  }

  async put<TData = any, TBody = any>(
    endpoint: string,
    body?: TBody,
    options?: Omit<RequestOptions<TBody>, 'method' | 'body'>
  ): Promise<APIResponse<TData>> {
    return this.request<TData, TBody>(endpoint, { ...options, method: 'PUT', body });
  }

  async patch<TData = any, TBody = any>(
    endpoint: string,
    body?: TBody,
    options?: Omit<RequestOptions<TBody>, 'method' | 'body'>
  ): Promise<APIResponse<TData>> {
    return this.request<TData, TBody>(endpoint, { ...options, method: 'PATCH', body });
  }

  async delete<TData = any>(
    endpoint: string,
    options?: Omit<RequestOptions, 'method' | 'body'>
  ): Promise<APIResponse<TData>> {
    return this.request<TData>(endpoint, { ...options, method: 'DELETE' });
  }
}

// Create singleton instance
let apiClientInstance: APIClient | null = null;

export function createAPIClient(config?: APIConfig): APIClient {
  apiClientInstance = new APIClient(config);
  return apiClientInstance;
}

export function getAPIClient(): APIClient {
  if (!apiClientInstance) {
    apiClientInstance = new APIClient();
  }
  return apiClientInstance;
}

export default APIClient;
```

### 4. Generate useAPI Hook (Query)

```typescript
// hooks/useAPI/useAPI.ts

import { useState, useEffect, useCallback, useRef } from 'react';
import { getAPIClient } from './apiClient';
import type { RequestOptions, APIHookResult } from './types';

/**
 * React hook for making API requests (similar to useQuery)
 *
 * @param endpoint - API endpoint to call
 * @param options - Request options
 * @param dependencies - Array of dependencies that trigger refetch
 * @param enabled - Whether to automatically fetch on mount (default: true)
 */
export function useAPI<TData = any>(
  endpoint: string | null,
  options: Omit<RequestOptions, 'method' | 'body'> = {},
  dependencies: any[] = [],
  enabled: boolean = true
): APIHookResult<TData> {
  const [data, setData] = useState<TData | null>(null);
  const [loading, setLoading] = useState<boolean>(enabled);
  const [error, setError] = useState<any>(null);

  const abortControllerRef = useRef<AbortController | null>(null);
  const isMountedRef = useRef(true);

  const fetchData = useCallback(async () => {
    if (!endpoint) {
      setLoading(false);
      return;
    }

    try {
      // Cancel previous request
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }

      // Create new abort controller
      abortControllerRef.current = new AbortController();

      setLoading(true);
      setError(null);

      const apiClient = getAPIClient();
      const response = await apiClient.get<TData>(endpoint, {
        ...options,
        signal: abortControllerRef.current.signal,
      });

      if (isMountedRef.current) {
        setData(response.data);
        setLoading(false);
      }
    } catch (err: any) {
      if (isMountedRef.current && err.message !== 'Request timeout or cancelled') {
        setError(err);
        setLoading(false);
      }
    }
  }, [endpoint, ...dependencies]);

  const reset = useCallback(() => {
    setData(null);
    setError(null);
    setLoading(false);
  }, []);

  // Fetch on mount and when dependencies change
  useEffect(() => {
    if (enabled) {
      fetchData();
    }

    return () => {
      isMountedRef.current = false;
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, [fetchData, enabled]);

  return {
    data,
    loading,
    error,
    refetch: fetchData,
    reset,
  };
}
```

### 5. Generate useMutation Hook

```typescript
// hooks/useAPI/useMutation.ts

import { useState, useCallback, useRef } from 'react';
import { getAPIClient } from './apiClient';
import type { RequestOptions, MutationResult, HTTPMethod } from './types';

/**
 * React hook for making API mutations (POST, PUT, DELETE, PATCH)
 *
 * @param endpoint - API endpoint to call
 * @param method - HTTP method (POST, PUT, DELETE, PATCH)
 * @param options - Request options
 */
export function useMutation<TData = any, TVariables = any>(
  endpoint: string,
  method: Exclude<HTTPMethod, 'GET'> = 'POST',
  options: Omit<RequestOptions, 'method' | 'body'> = {}
): MutationResult<TData, TVariables> {
  const [data, setData] = useState<TData | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<any>(null);

  const abortControllerRef = useRef<AbortController | null>(null);

  const mutate = useCallback(async (variables?: TVariables): Promise<TData> => {
    try {
      // Cancel previous request
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }

      // Create new abort controller
      abortControllerRef.current = new AbortController();

      setLoading(true);
      setError(null);

      const apiClient = getAPIClient();
      const response = await apiClient.request<TData, TVariables>(endpoint, {
        ...options,
        method,
        body: variables,
        signal: abortControllerRef.current.signal,
      });

      setData(response.data);
      setLoading(false);
      return response.data;
    } catch (err: any) {
      setError(err);
      setLoading(false);
      throw err;
    }
  }, [endpoint, method, JSON.stringify(options)]);

  const reset = useCallback(() => {
    setData(null);
    setError(null);
    setLoading(false);
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }
  }, []);

  return {
    mutate,
    data,
    loading,
    error,
    reset,
  };
}
```

### 6. Generate API Context Provider

```typescript
// contexts/APIContext.tsx

import React, { createContext, useContext, useMemo } from 'react';
import { createAPIClient } from '../hooks/useAPI/apiClient';
import type { APIConfig } from '../hooks/useAPI/types';
import type APIClient from '../hooks/useAPI/apiClient';

interface APIContextValue {
  client: APIClient;
}

const APIContext = createContext<APIContextValue | null>(null);

interface APIProviderProps {
  config?: APIConfig;
  children: React.ReactNode;
}

export const APIProvider: React.FC<APIProviderProps> = ({ config, children }) => {
  const client = useMemo(() => createAPIClient(config), [JSON.stringify(config)]);

  const value = useMemo(() => ({ client }), [client]);

  return <APIContext.Provider value={value}>{children}</APIContext.Provider>;
};

export function useAPIContext(): APIContextValue {
  const context = useContext(APIContext);
  if (!context) {
    throw new Error('useAPIContext must be used within APIProvider');
  }
  return context;
}
```

### 7. Generate Setup and Configuration

```typescript
// config/apiConfig.ts

import type { APIConfig } from '../hooks/useAPI/types';

const isDev = process.env.NODE_ENV === 'development';

export const apiConfig: APIConfig = {
  baseURL: isDev
    ? process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000'
    : process.env.REACT_APP_API_BASE_URL || 'https://api.example.com',
  headers: {
    'Content-Type': 'application/json',
    // Add default headers here
  },
  timeout: 30000, // 30 seconds
  retryAttempts: 3,
  retryDelay: 1000, // 1 second
  enableCache: true,
  cacheDuration: 5 * 60 * 1000, // 5 minutes
};

export default apiConfig;
```

### 8. Generate Usage Examples

```typescript
// examples/APIUsageExamples.tsx

import React, { useState } from 'react';
import { useAPI } from '../hooks/useAPI/useAPI';
import { useMutation } from '../hooks/useAPI/useMutation';

// ============================================================================
// Example 1: Simple GET Request
// ============================================================================

interface User {
  id: number;
  name: string;
  email: string;
}

function UserProfile({ userId }: { userId: number }) {
  const { data, loading, error, refetch } = useAPI<User>(
    `/users/${userId}`,
    {}, // options
    [userId] // dependencies
  );

  if (loading) return <div>Loading...</div>;
  if (error) return <div>Error: {error.message}</div>;
  if (!data) return null;

  return (
    <div>
      <h2>{data.name}</h2>
      <p>{data.email}</p>
      <button onClick={refetch}>Refresh</button>
    </div>
  );
}

// ============================================================================
// Example 2: GET with Query Parameters
// ============================================================================

interface Post {
  id: number;
  title: string;
  body: string;
}

function PostList({ page = 1, limit = 10 }: { page?: number; limit?: number }) {
  const { data, loading, error } = useAPI<Post[]>(
    '/posts',
    {
      params: { page, limit },
      cache: true, // Enable caching for this request
    },
    [page, limit]
  );

  if (loading) return <div>Loading posts...</div>;
  if (error) return <div>Error: {error.message}</div>;

  return (
    <ul>
      {data?.map(post => (
        <li key={post.id}>
          <h3>{post.title}</h3>
          <p>{post.body}</p>
        </li>
      ))}
    </ul>
  );
}

// ============================================================================
// Example 3: POST Mutation (Create)
// ============================================================================

interface CreateUserData {
  name: string;
  email: string;
  password: string;
}

function CreateUserForm() {
  const [formData, setFormData] = useState<CreateUserData>({
    name: '',
    email: '',
    password: '',
  });

  const { mutate, loading, error, data } = useMutation<User, CreateUserData>(
    '/users',
    'POST'
  );

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const newUser = await mutate(formData);
      console.log('User created:', newUser);
      alert('User created successfully!');
    } catch (err) {
      console.error('Failed to create user:', err);
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <input
        type="text"
        placeholder="Name"
        value={formData.name}
        onChange={e => setFormData({ ...formData, name: e.target.value })}
      />
      <input
        type="email"
        placeholder="Email"
        value={formData.email}
        onChange={e => setFormData({ ...formData, email: e.target.value })}
      />
      <input
        type="password"
        placeholder="Password"
        value={formData.password}
        onChange={e => setFormData({ ...formData, password: e.target.value })}
      />
      <button type="submit" disabled={loading}>
        {loading ? 'Creating...' : 'Create User'}
      </button>
      {error && <div className="error">{error.message}</div>}
      {data && <div className="success">User created: {data.name}</div>}
    </form>
  );
}

// ============================================================================
// Example 4: PUT Mutation (Update)
// ============================================================================

interface UpdateUserData {
  name?: string;
  email?: string;
}

function UpdateUserForm({ userId }: { userId: number }) {
  const [updates, setUpdates] = useState<UpdateUserData>({});

  const { mutate, loading } = useMutation<User, UpdateUserData>(
    `/users/${userId}`,
    'PUT'
  );

  const handleUpdate = async () => {
    try {
      const updatedUser = await mutate(updates);
      alert(`User updated: ${updatedUser.name}`);
    } catch (err) {
      alert('Failed to update user');
    }
  };

  return (
    <div>
      <input
        type="text"
        placeholder="New name"
        onChange={e => setUpdates({ ...updates, name: e.target.value })}
      />
      <button onClick={handleUpdate} disabled={loading}>
        Update
      </button>
    </div>
  );
}

// ============================================================================
// Example 5: DELETE Mutation
// ============================================================================

function DeleteUserButton({ userId }: { userId: number }) {
  const { mutate, loading } = useMutation<void, void>(
    `/users/${userId}`,
    'DELETE'
  );

  const handleDelete = async () => {
    if (!window.confirm('Are you sure?')) return;

    try {
      await mutate();
      alert('User deleted successfully');
    } catch (err) {
      alert('Failed to delete user');
    }
  };

  return (
    <button onClick={handleDelete} disabled={loading}>
      {loading ? 'Deleting...' : 'Delete User'}
    </button>
  );
}

// ============================================================================
// Example 6: Conditional Fetching
// ============================================================================

function ConditionalData({ shouldFetch }: { shouldFetch: boolean }) {
  const { data, loading } = useAPI<any>(
    '/data',
    {},
    [],
    shouldFetch // Only fetch when shouldFetch is true
  );

  if (!shouldFetch) return <div>Fetching disabled</div>;
  if (loading) return <div>Loading...</div>;

  return <div>Data: {JSON.stringify(data)}</div>;
}

// ============================================================================
// Example 7: With Custom Headers (Authentication)
// ============================================================================

function AuthenticatedRequest() {
  const token = localStorage.getItem('authToken');

  const { data, loading, error } = useAPI<any>(
    '/protected-resource',
    {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    }
  );

  if (loading) return <div>Loading...</div>;
  if (error) return <div>Error: {error.message}</div>;

  return <div>Protected data: {JSON.stringify(data)}</div>;
}

// ============================================================================
// Example 8: File Upload
// ============================================================================

function FileUpload() {
  const [file, setFile] = useState<File | null>(null);

  const { mutate, loading, error } = useMutation<any, FormData>(
    '/upload',
    'POST',
    {
      headers: {
        // Don't set Content-Type for FormData (browser sets it automatically)
        'Content-Type': 'multipart/form-data',
      },
    }
  );

  const handleUpload = async () => {
    if (!file) return;

    const formData = new FormData();
    formData.append('file', file);

    try {
      const result = await mutate(formData);
      alert('File uploaded successfully');
    } catch (err) {
      alert('Upload failed');
    }
  };

  return (
    <div>
      <input type="file" onChange={e => setFile(e.target.files?.[0] || null)} />
      <button onClick={handleUpload} disabled={loading || !file}>
        {loading ? 'Uploading...' : 'Upload'}
      </button>
      {error && <div>Error: {error.message}</div>}
    </div>
  );
}

export {
  UserProfile,
  PostList,
  CreateUserForm,
  UpdateUserForm,
  DeleteUserButton,
  ConditionalData,
  AuthenticatedRequest,
  FileUpload,
};
```

### 9. Generate App Setup with Provider

```typescript
// App.tsx

import React from 'react';
import { APIProvider } from './contexts/APIContext';
import apiConfig from './config/apiConfig';
import { getAPIClient } from './hooks/useAPI/apiClient';

// Setup interceptors
const apiClient = getAPIClient();

// Request interceptor - Add auth token
apiClient.addRequestInterceptor(async (config) => {
  const token = localStorage.getItem('authToken');
  if (token) {
    return {
      ...config,
      headers: {
        ...config.headers,
        Authorization: `Bearer ${token}`,
      },
    };
  }
  return config;
});

// Response interceptor - Transform data
apiClient.addResponseInterceptor(async (response) => {
  // You can transform response data here
  console.log('Response received:', response.status);
  return response;
});

// Error interceptor - Handle auth errors
apiClient.addErrorInterceptor(async (error) => {
  if (error.status === 401) {
    // Handle unauthorized - redirect to login
    localStorage.removeItem('authToken');
    window.location.href = '/login';
  }
  return error;
});

function App() {
  return (
    <APIProvider config={apiConfig}>
      <div className="App">
        {/* Your app components */}
      </div>
    </APIProvider>
  );
}

export default App;
```

### 10. Generate Tests

```typescript
// hooks/useAPI/__tests__/useAPI.test.ts

import { renderHook, waitFor } from '@testing-library/react';
import { useAPI } from '../useAPI';
import { createAPIClient } from '../apiClient';

// Mock fetch
global.fetch = jest.fn();

describe('useAPI', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    createAPIClient({ baseURL: 'https://api.test.com' });
  });

  it('fetches data successfully', async () => {
    const mockData = { id: 1, name: 'Test' };
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      status: 200,
      json: async () => mockData,
      headers: new Headers(),
    });

    const { result } = renderHook(() => useAPI('/test'));

    expect(result.current.loading).toBe(true);

    await waitFor(() => {
      expect(result.current.loading).toBe(false);
    });

    expect(result.current.data).toEqual(mockData);
    expect(result.current.error).toBeNull();
  });

  it('handles errors', async () => {
    (global.fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

    const { result } = renderHook(() => useAPI('/test'));

    await waitFor(() => {
      expect(result.current.loading).toBe(false);
    });

    expect(result.current.error).toBeTruthy();
    expect(result.current.data).toBeNull();
  });

  it('refetches data', async () => {
    const mockData = { id: 1, name: 'Test' };
    (global.fetch as jest.Mock).mockResolvedValue({
      ok: true,
      status: 200,
      json: async () => mockData,
      headers: new Headers(),
    });

    const { result } = renderHook(() => useAPI('/test'));

    await waitFor(() => {
      expect(result.current.loading).toBe(false);
    });

    result.current.refetch();

    expect(result.current.loading).toBe(true);

    await waitFor(() => {
      expect(result.current.loading).toBe(false);
    });

    expect(global.fetch).toHaveBeenCalledTimes(2);
  });
});
```

### 11. Generate Documentation

```markdown
# API Integration Hook

A comprehensive React hook system for API integration with TypeScript support.

## Features

- ✅ Support for all HTTP methods (GET, POST, PUT, DELETE, PATCH)
- ✅ Automatic retry with exponential backoff
- ✅ Request cancellation on unmount
- ✅ Response caching
- ✅ Loading, error, and data states
- ✅ Request/response interceptors
- ✅ TypeScript generics for type safety
- ✅ Timeout handling
- ✅ Query parameters support
- ✅ Custom headers
- ✅ File upload support

## Installation

No external dependencies required! Uses native `fetch` API.

## Setup

1. Wrap your app with `APIProvider`:

```tsx
import { APIProvider } from './contexts/APIContext';
import apiConfig from './config/apiConfig';

function App() {
  return (
    <APIProvider config={apiConfig}>
      <YourApp />
    </APIProvider>
  );
}
```

2. Configure in `config/apiConfig.ts`:

```typescript
export const apiConfig: APIConfig = {
  baseURL: 'https://api.example.com',
  timeout: 30000,
  retryAttempts: 3,
  retryDelay: 1000,
  enableCache: true,
  cacheDuration: 300000,
};
```

## Usage

### GET Request

```typescript
const { data, loading, error, refetch } = useAPI<User>('/users/1');
```

### POST Request

```typescript
const { mutate, loading, error } = useMutation<User, CreateUserData>(
  '/users',
  'POST'
);

await mutate({ name: 'John', email: 'john@example.com' });
```

### With Query Parameters

```typescript
const { data } = useAPI('/posts', {
  params: { page: 1, limit: 10 }
});
```

### With Custom Headers

```typescript
const { data } = useAPI('/protected', {
  headers: { Authorization: `Bearer ${token}` }
});
```

### Conditional Fetching

```typescript
const { data } = useAPI(
  '/data',
  {},
  [],
  shouldFetch // Only fetch when true
);
```

## API Reference

### `useAPI<TData>(endpoint, options, dependencies, enabled)`

Query hook for GET requests.

**Parameters:**
- `endpoint`: API endpoint
- `options`: Request options
- `dependencies`: Dependency array for refetching
- `enabled`: Whether to auto-fetch (default: true)

**Returns:**
- `data`: Response data
- `loading`: Loading state
- `error`: Error object
- `refetch`: Manual refetch function
- `reset`: Reset state function

### `useMutation<TData, TVariables>(endpoint, method, options)`

Mutation hook for POST/PUT/DELETE/PATCH.

**Parameters:**
- `endpoint`: API endpoint
- `method`: HTTP method
- `options`: Request options

**Returns:**
- `mutate`: Function to trigger mutation
- `data`: Response data
- `loading`: Loading state
- `error`: Error object
- `reset`: Reset state function

## Advanced Features

### Interceptors

```typescript
const apiClient = getAPIClient();

// Request interceptor
apiClient.addRequestInterceptor(async (config) => {
  config.headers = { ...config.headers, 'X-Custom': 'value' };
  return config;
});

// Response interceptor
apiClient.addResponseInterceptor(async (response) => {
  console.log('Response:', response);
  return response;
});

// Error interceptor
apiClient.addErrorInterceptor(async (error) => {
  if (error.status === 401) {
    // Handle unauthorized
  }
  return error;
});
```

### Cache Management

```typescript
const apiClient = getAPIClient();

// Clear all cache
apiClient.clearCache();

// Clear specific pattern
apiClient.clearCache('/users');
```

## Error Handling

All errors include:
- `message`: Error message
- `status`: HTTP status code
- `statusText`: HTTP status text
- `details`: Additional error details
- `timestamp`: Error timestamp
```

## Deliverables

When complete, provide:

1. **Type definitions** (`types.ts`) - Complete TypeScript interfaces
2. **API Client class** (`apiClient.ts`) - Core HTTP client with retry logic
3. **useAPI hook** (`useAPI.ts`) - Query hook for GET requests
4. **useMutation hook** (`useMutation.ts`) - Mutation hook for POST/PUT/DELETE
5. **API Context** (`APIContext.tsx`) - Provider for global configuration
6. **Configuration** (`apiConfig.ts`) - Default settings
7. **Examples** (`APIUsageExamples.tsx`) - 8+ usage examples
8. **Tests** (`useAPI.test.ts`) - Unit tests
9. **Documentation** (`README.md`) - Complete API reference

## Features Summary

- ✅ **All HTTP Methods**: GET, POST, PUT, DELETE, PATCH
- ✅ **Retry Logic**: Exponential backoff (configurable)
- ✅ **State Management**: Loading, error, data states
- ✅ **Request Cancellation**: Automatic cleanup on unmount
- ✅ **Caching**: Optional response caching with TTL
- ✅ **Interceptors**: Request, response, and error interceptors
- ✅ **TypeScript**: Full type safety with generics
- ✅ **Timeout Handling**: Configurable timeouts
- ✅ **Query Parameters**: Automatic URL building
- ✅ **Custom Headers**: Per-request or global headers
- ✅ **Error Handling**: Comprehensive error objects
- ✅ **File Upload**: FormData support
- ✅ **Conditional Fetching**: Enable/disable fetching
- ✅ **No Dependencies**: Uses native fetch API

## Configuration Options

```typescript
{
  baseURL: string;              // Base API URL
  headers: Record<string, string>; // Default headers
  timeout: number;              // Request timeout (ms)
  retryAttempts: number;        // Max retry attempts
  retryDelay: number;           // Initial retry delay (ms)
  enableCache: boolean;         // Enable response caching
  cacheDuration: number;        // Cache TTL (ms)
}
```
