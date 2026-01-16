import { useCallback } from 'react';

// Simple API hook for demonstration purposes
// In a real application, you would use a library like axios or fetch with proper error handling

// Handle both browser and server environments
const API_BASE_URL = typeof process !== 'undefined' && process.env
  ? process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000'
  : 'http://localhost:8000';  // Default fallback

const useApi = () => {
  const request = useCallback(async (endpoint, options = {}) => {
    const url = `${API_BASE_URL}${endpoint}`;

    const config = {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    };

    // Add auth token if available
    const token = localStorage.getItem('token');
    if (token && !config.headers.Authorization) {
      config.headers.Authorization = `Bearer ${token}`;
    }

    try {
      const response = await fetch(url, config);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('API request failed:', error);
      throw error;
    }
  }, []);

  const get = useCallback((endpoint, options = {}) => {
    return request(endpoint, { ...options, method: 'GET' });
  }, [request]);

  const post = useCallback((endpoint, data, options = {}) => {
    return request(endpoint, {
      ...options,
      method: 'POST',
      body: JSON.stringify(data),
    });
  }, [request]);

  const put = useCallback((endpoint, data, options = {}) => {
    return request(endpoint, {
      ...options,
      method: 'PUT',
      body: JSON.stringify(data),
    });
  }, [request]);

  const del = useCallback((endpoint, options = {}) => {
    return request(endpoint, {
      ...options,
      method: 'DELETE',
    });
  }, [request]);

  return { request, get, post, put, del };
};

export { useApi };