// Global Chat Injector Plugin for Docusaurus
// This plugin injects the global chat panel into all pages

import React from 'react';

const GlobalChatInjector = () => {
  // This component will be injected into all pages
  return (
    <div id="global-chat-root" style={{ position: 'fixed', bottom: 0, right: 0, zIndex: 1000 }}>
      {/* The global chat panel will be mounted here */}
    </div>
  );
};

export default GlobalChatInjector;