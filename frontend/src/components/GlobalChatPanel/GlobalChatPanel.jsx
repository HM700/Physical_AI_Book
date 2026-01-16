import React, { useState, useEffect } from 'react';
import ChatPanel from '../ChatPanel/ChatPanel';

// Global chat panel component that appears on all pages
const GlobalChatPanel = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);

  // Toggle chat panel visibility
  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
    if (!isChatOpen) {
      setIsMinimized(false);
    }
  };

  // Minimize/maximize chat panel
  const toggleMinimize = () => {
    setIsMinimized(!isMinimized);
  };

  // Close chat panel
  const closeChat = () => {
    setIsChatOpen(false);
    setIsMinimized(true);
  };

  return (
    <>
      {!isChatOpen ? (
        // Floating chat button
        <button
          onClick={toggleChat}
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: '1000',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#2563eb',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'all 0.3s ease',
          }}
          aria-label="Open AI Tutor Chat"
        >
          💬
        </button>
      ) : isMinimized ? (
        // Minimized chat button
        <button
          onClick={toggleMinimize}
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: '1000',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#2563eb',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'all 0.3s ease',
          }}
          aria-label="Expand AI Tutor Chat"
        >
          💬
        </button>
      ) : (
        // Full chat panel
        <div
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: '1000',
            width: '400px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 10px 25px rgba(0,0,0,0.15)',
            display: 'flex',
            flexDirection: 'column',
            border: '1px solid #e2e8f0',
          }}
        >
          <div
            style={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              padding: '12px 16px',
              backgroundColor: '#2563eb',
              color: 'white',
              borderTopLeftRadius: '12px',
              borderTopRightRadius: '12px',
            }}
          >
            <h4 style={{ margin: 0, fontSize: '16px' }}>AI Tutor</h4>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={toggleMinimize}
                style={{
                  background: 'none',
                  border: 'none',
                  color: 'white',
                  cursor: 'pointer',
                  padding: '4px',
                  borderRadius: '4px',
                }}
                aria-label="Minimize chat"
              >
                −
              </button>
              <button
                onClick={closeChat}
                style={{
                  background: 'none',
                  border: 'none',
                  color: 'white',
                  cursor: 'pointer',
                  padding: '4px',
                  borderRadius: '4px',
                }}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>
          </div>
          <div style={{ flex: 1, overflow: 'hidden' }}>
            <ChatPanel embedded={true} />
          </div>
        </div>
      )}
    </>
  );
};

export default GlobalChatPanel;