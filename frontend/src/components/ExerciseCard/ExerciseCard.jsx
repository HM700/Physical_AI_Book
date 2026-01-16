import React from 'react';
import clsx from 'clsx';
import styles from './ExerciseCard.module.css';

const ExerciseCard = ({ title, difficulty, duration, type = "exercise", children, onStart }) => {
  const getTypeStyles = (type) => {
    switch (type) {
      case 'project':
        return { bg: '#ebf4ff', text: '#4c51bf', border: '#4c51bf' };
      case 'challenge':
        return { bg: '#fed7d7', text: '#c53030', border: '#c53030' };
      case 'exercise':
      default:
        return { bg: '#e6fffa', text: '#38a169', border: '#38a169' };
    }
  };

  const typeStyles = getTypeStyles(type);

  return (
    <div
      className={clsx(styles.exerciseCard, styles[type])}
      style={{ borderLeftColor: typeStyles.border }}
    >
      <div className={styles.exerciseHeader}>
        <div className={styles.exerciseMeta}>
          <span
            className={clsx(styles.badge, styles.difficulty)}
            style={{
              backgroundColor: typeStyles.bg,
              color: typeStyles.text
            }}
          >
            {difficulty}
          </span>
          <span
            className={clsx(styles.badge, styles.duration)}
            style={{
              backgroundColor: typeStyles.bg,
              color: typeStyles.text
            }}
          >
            ⏱️ {duration}
          </span>
        </div>
        <h3 className={styles.exerciseTitle}>
          <span className={styles.exerciseIcon}>🎯</span>
          {title}
        </h3>
      </div>

      <div className={styles.exerciseContent}>
        {children}
      </div>

      <div className={styles.exerciseFooter}>
        <button
          className={styles.startButton}
          onClick={onStart}
        >
          Start {type.charAt(0).toUpperCase() + type.slice(1)}
        </button>
        <span
          className={clsx(styles.typeIndicator, styles[type])}
          style={{
            backgroundColor: typeStyles.bg,
            color: typeStyles.text
          }}
        >
          {type === 'project' ? '🏆 Project' :
           type === 'challenge' ? '🔥 Challenge' : '📝 Exercise'}
        </span>
      </div>
    </div>
  );
};

export default ExerciseCard;