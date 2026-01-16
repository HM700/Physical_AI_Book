import React, { useState, useEffect } from 'react';
import { useApi } from '../hooks/useApi';

const Chatbot = ({ sessionId = null }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const { post } = useApi();

  // Initialize chat session
  useEffect(() => {
    // Load existing messages if session exists
    if (sessionId) {
      loadSessionMessages();
    }
  }, [sessionId]);

  const loadSessionMessages = async () => {
    try {
      // In a real implementation, we would fetch messages from the backend
      // const response = await get(`/chat/sessions/${sessionId}/messages`);
      // setMessages(response.data);
    } catch (error) {
      console.error('Failed to load messages:', error);
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    // Add user message to UI immediately
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Send message to backend
      const response = await post('/chat/chat', {
        message: inputValue,
        session_id: sessionId
      });

      const botMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: response.data.message,
        timestamp: response.data.timestamp
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Failed to send message:', error);

      // Add error message to UI
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chat-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message message-${message.role}`}
            style={{
              textAlign: message.role === 'user' ? 'right' : 'left',
              marginBottom: '1rem'
            }}
          >
            <div
              style={{
                display: 'inline-block',
                padding: '0.5rem 1rem',
                borderRadius: '8px',
                backgroundColor: message.role === 'user' ? '#e3f2fd' : '#f5f5f5',
                maxWidth: '80%'
              }}
            >
              <div style={{ fontSize: '0.9rem' }}>{message.content}</div>
              <div style={{ fontSize: '0.7rem', opacity: 0.7, marginTop: '0.25rem' }}>
                {new Date(message.timestamp).toLocaleTimeString()}
              </div>
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="message message-assistant" style={{ textAlign: 'left', marginBottom: '1rem' }}>
            <div
              style={{
                display: 'inline-block',
                padding: '0.5rem 1rem',
                borderRadius: '8px',
                backgroundColor: '#f5f5f5',
                maxWidth: '80%'
              }}
            >
              <div>Thinking...</div>
            </div>
          </div>
        )}
      </div>

      <form onSubmit={handleSubmit} className="chat-input-form" style={{ marginTop: '1rem' }}>
        <div style={{ display: 'flex', gap: '0.5rem' }}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask about Physical AI concepts..."
            disabled={isLoading}
            style={{
              flex: 1,
              padding: '0.5rem',
              border: '1px solid #ddd',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
          <button
            type="submit"
            disabled={!inputValue.trim() || isLoading}
            style={{
              padding: '0.5rem 1rem',
              backgroundColor: '#2e8555',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: !inputValue.trim() || isLoading ? 'not-allowed' : 'pointer',
              opacity: !inputValue.trim() || isLoading ? 0.6 : 1
            }}
          >
            Send
          </button>
        </div>
      </form>
    </div>
  );
};

export default Chatbot;