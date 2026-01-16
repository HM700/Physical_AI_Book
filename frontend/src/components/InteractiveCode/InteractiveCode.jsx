import React, { useState, useRef } from 'react';
import clsx from 'clsx';
import styles from './InteractiveCode.module.css';

const InteractiveCode = ({ children, language = 'python', title = 'Code Example', hint = '' }) => {
  const [copied, setCopied] = useState(false);
  const [expanded, setExpanded] = useState(false);
  const codeRef = useRef(null);

  const copyToClipboard = () => {
    if (codeRef.current) {
      navigator.clipboard.writeText(codeRef.current.textContent);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    }
  };

  const toggleExpand = () => {
    setExpanded(!expanded);
  };

  return (
    <div className={styles.interactiveCode}>
      <div className={styles.codeHeader}>
        <span className={styles.codeTitle}>{title} ({language})</span>
        <div className={styles.codeActions}>
          <button
            className={styles.copyButton}
            onClick={copyToClipboard}
            disabled={copied}
          >
            {copied ? '✓ Copied!' : 'Copy'}
          </button>
          {children.length > 100 && (
            <button
              className={styles.expandButton}
              onClick={toggleExpand}
            >
              {expanded ? 'Collapse' : 'Expand'}
            </button>
          )}
        </div>
      </div>
      <pre className={styles.codeBlock}>
        <code className={`language-${language}`} ref={codeRef}>
          {expanded ? children :
           children.length > 100 ? children.substring(0, 100) + '...' : children}
        </code>
      </pre>
      {hint && (
        <div className={styles.codeHint}>
          💡 {hint}
        </div>
      )}
    </div>
  );
};

export default InteractiveCode;