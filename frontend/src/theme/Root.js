import React from 'react';
import GlobalChatPanel from '../components/GlobalChatPanel/GlobalChatPanel';

// Custom root component that adds the global chat panel to all pages
export default function Root({ children }) {
  return (
    <>
      {children}
      <GlobalChatPanel />
    </>
  );
}