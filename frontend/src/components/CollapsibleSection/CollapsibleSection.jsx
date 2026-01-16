import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CollapsibleSection.module.css';

const CollapsibleSection = ({ title, children, defaultOpen = false, icon = null }) => {
  const [isOpen, setIsOpen] = useState(defaultOpen);

  const toggleOpen = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.collapsibleSection}>
      <button
        className={clsx(styles.sectionHeader, { [styles.isOpen]: isOpen })}
        onClick={toggleOpen}
        aria-expanded={isOpen}
        aria-controls={`collapsible-content-${title.replace(/\s+/g, '-')}`}
      >
        <div className={styles.sectionTitle}>
          {icon && <span className={styles.sectionIcon}>{icon}</span>}
          {title}
        </div>
        <span className={clsx(styles.arrow, { [styles.rotated]: isOpen })}>
          ▼
        </span>
      </button>

      {isOpen && (
        <div
          id={`collapsible-content-${title.replace(/\s+/g, '-')}`}
          className={styles.sectionContent}
        >
          {children}
        </div>
      )}
    </div>
  );
};

export default CollapsibleSection;