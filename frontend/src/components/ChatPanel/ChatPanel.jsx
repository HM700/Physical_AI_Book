import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ChatPanel.module.css';

const ChatPanel = ({ initialOpen = false, embedded = false }) => {
  const [isOpen, setIsOpen] = useState(embedded ? true : initialOpen);
  const [messages, setMessages] = useState([
    {
      id: 1,
      role: 'assistant',
      content: 'Hello! I\'m your Physical AI tutor. How can I help you with robotics, ROS 2, simulation, or AI concepts today?',
      timestamp: new Date(Date.now() - 300000).toISOString()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const formatTime = (timestamp) => {
    return new Date(timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue.trim(),
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Simulate API call to backend/RAG service
      // In a real implementation, this would call the backend API
      await new Promise(resolve => setTimeout(resolve, 1000)); // Simulate API delay

      // Generate a simulated response based on the user's query
      const responseContent = generateSimulatedResponse(userMessage.content);

      const botMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: responseContent,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const generateSimulatedResponse = (query) => {
    const lowerQuery = query.toLowerCase();

    if (lowerQuery.includes('hello') || lowerQuery.includes('hi') || lowerQuery.includes('hey')) {
      return 'Hello! I\'m your Physical AI tutor. I can help you with concepts related to ROS 2, Gazebo simulation, NVIDIA Isaac, and Vision-Language-Action systems. What would you like to learn about?';
    } else if (lowerQuery.includes('ros') || lowerQuery.includes('ros 2')) {
      return 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. In the context of Physical AI, ROS 2 serves as the middleware that allows AI software to communicate with and control humanoid robots in simulated environments.';
    } else if (lowerQuery.includes('gazebo') || lowerQuery.includes('simulation')) {
      return 'Gazebo is a 3D simulation environment that enables accurate and efficient simulation of robots in complex indoor and outdoor environments. In the Physical AI curriculum, Gazebo is used as part of the Digital Twin module (Module 2) to create digital replicas of humanoid robots and their environments. This allows you to validate control logic, physics interactions, and sensor behavior in a safe, controlled environment before deploying to real hardware.';
    } else if (lowerQuery.includes('isaac') || lowerQuery.includes('nvidia')) {
      return 'NVIDIA Isaac is a comprehensive robotics platform that includes Isaac Sim for photorealistic simulation and Isaac ROS for accelerated perception and navigation. Isaac Sim provides advanced physics simulation and synthetic data generation capabilities, while Isaac ROS offers hardware-accelerated perception and navigation components. These tools are central to Module 3 (AI-Robot Brain) where you\'ll learn to build the AI brain for humanoid robots.';
    } else if (lowerQuery.includes('vision') || lowerQuery.includes('language') || lowerQuery.includes('action') || lowerQuery.includes('vla')) {
      return 'Vision-Language-Action (VLA) systems represent the integration of computer vision, natural language processing, and robotic action execution. In Module 4 of the Physical AI curriculum, you\'ll learn how to integrate LLMs, speech recognition, and cognitive planning to enable humanoid robots to perform tasks based on human instructions. This involves mapping natural language commands to robot actions using advanced AI techniques.';
    } else if (lowerQuery.includes('module') || lowerQuery.includes('learn')) {
      return 'The Physical AI Book curriculum consists of four progressive modules:\n\n1. **Module 1: ROS 2 Fundamentals** - Learn the foundational middleware layer for robot control\n2. **Module 2: Digital Twin** - Create simulation environments for robot validation\n3. **Module 3: AI-Robot Brain** - Build AI brains using NVIDIA Isaac Sim and Isaac ROS\n4. **Module 4: Vision-Language-Action** - Integrate LLMs for natural human-robot interaction\n\nEach module builds upon the previous one, following a simulation-first approach.';
    } else {
      return 'I understand you\'re asking about: "' + query + '". As your Physical AI tutor, I can help explain concepts related to robotics, AI, and simulation. The Physical AI Book curriculum covers ROS 2, Digital Twins with Gazebo, AI-Robot Brains with NVIDIA Isaac, and Vision-Language-Action systems. Could you be more specific about what you\'d like to learn?';
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  // If embedded (no header needed), render just the content
  if (embedded) {
    return (
      <div className={styles.chatEmbeddedContent}>
        <div className={styles.chatMessages}>
          {messages.map((message) => (
            <div
              key={message.id}
              className={clsx(styles.message, styles[message.role])}
            >
              <div className={styles.messageContent}>
                {message.content.split('\n').map((line, i) => (
                  <p key={i} style={{ margin: '0.5em 0', color: message.role === 'user' ? 'white' : 'black' }}>{line}</p>
                ))}
              </div>
              <div className={styles.messageTimestamp}>
                {formatTime(message.timestamp)}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className={clsx(styles.message, styles.assistant)}>
              <div className={styles.typingIndicator}>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
                <span style={{ color: 'black' }}>AI Tutor is thinking...</span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className={styles.chatInputForm}>
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask about Physical AI concepts..."
            className={styles.chatInput}
            aria-label="Type your message"
            disabled={isLoading}
          />
          <button
            type="submit"
            className={styles.chatSubmitButton}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            Send
          </button>
        </form>
      </div>
    );
  }

  // Non-embedded mode: handle opening/closing with toggle button
  if (!isOpen) {
    return (
      <button
        className={styles.chatToggle}
        onClick={toggleChat}
        aria-label="Open AI Tutor Chat"
      >
        💬 AI Tutor
      </button>
    );
  }

  return (
    <div className={styles.chatContainer}>
      <div className={styles.chatPanel}>
        <div className={styles.chatHeader}>
          <h4 className={styles.chatTitle}>AI Tutor</h4>
          <button
            className={styles.closeButton}
            onClick={closeChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        <div className={styles.chatMessages}>
          {messages.map((message) => (
            <div
              key={message.id}
              className={clsx(styles.message, styles[message.role])}
            >
              <div className={styles.messageContent}>
                {message.content.split('\n').map((line, i) => (
                  <p key={i} style={{ margin: '0.5em 0', color: message.role === 'user' ? 'white' : 'black' }}>{line}</p>
                ))}
              </div>
              <div className={styles.messageTimestamp}>
                {formatTime(message.timestamp)}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className={clsx(styles.message, styles.assistant)}>
              <div className={styles.typingIndicator}>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
                <span style={{ color: 'black' }}>AI Tutor is thinking...</span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className={styles.chatInputForm}>
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask about Physical AI concepts..."
            className={styles.chatInput}
            aria-label="Type your message"
            disabled={isLoading}
          />
          <button
            type="submit"
            className={styles.chatSubmitButton}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            Send
          </button>
        </form>
      </div>
    </div>
  );
};

export default ChatPanel;