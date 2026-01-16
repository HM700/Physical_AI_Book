import React from 'react';
import clsx from 'clsx';
import styles from './BreadcrumbNav.module.css';

const BreadcrumbNav = ({ items = [], current }) => {
  return (
    <nav className={styles.breadcrumbNav} aria-label="Breadcrumb">
      <ol className={styles.breadcrumbList}>
        {items.map((item, index) => (
          <li key={index} className={styles.breadcrumbItem}>
            {item.url ? (
              <a
                href={item.url}
                className={styles.breadcrumbLink}
                aria-current={index === items.length - 1 ? 'page' : undefined}
              >
                {item.icon && <span className={styles.breadcrumbIcon}>{item.icon}</span>}
                {item.label}
              </a>
            ) : (
              <span
                className={clsx(styles.breadcrumbLink, styles.current)}
                aria-current="page"
              >
                {item.icon && <span className={styles.breadcrumbIcon}>{item.icon}</span>}
                {item.label}
              </span>
            )}
          </li>
        ))}
        {current && (
          <li className={styles.breadcrumbItem}>
            <span
              className={clsx(styles.breadcrumbLink, styles.current)}
              aria-current="page"
            >
              {current}
            </span>
          </li>
        )}
      </ol>
    </nav>
  );
};

export default BreadcrumbNav;