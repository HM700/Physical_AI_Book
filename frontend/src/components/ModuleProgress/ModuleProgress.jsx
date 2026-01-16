import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ModuleProgress.module.css';

// Mock data for modules - in a real implementation, this would come from props or API
const MODULES = [
  {
    id: 1,
    title: 'Module 1: ROS 2 Fundamentals',
    icon: '🤖',
    description: 'Learn the foundational middleware layer that allows AI software to control humanoid robots in simulated physical environments',
    difficulty: 'beginner',
    duration: '2-3 weeks',
    progress: 75,
    completed: false,
    lessons: 12,
    exercises: 5
  },
  {
    id: 2,
    title: 'Module 2: Digital Twin',
    icon: '🏗️',
    description: 'Create digital twins of humanoid robots and their environments to validate control logic, physics, and interactions',
    difficulty: 'intermediate',
    duration: '3-4 weeks',
    progress: 30,
    completed: false,
    lessons: 15,
    exercises: 7
  },
  {
    id: 3,
    title: 'Module 3: AI-Robot Brain',
    icon: '🧠',
    description: 'Build the AI brain for humanoid robots using NVIDIA Isaac Sim and Isaac ROS with photorealistic simulation',
    difficulty: 'intermediate',
    duration: '4-5 weeks',
    progress: 0,
    completed: false,
    lessons: 18,
    exercises: 9
  },
  {
    id: 4,
    title: 'Module 4: Vision-Language-Action',
    icon: '👁️',
    description: 'Integrate LLMs, speech recognition, and cognitive planning to enable humanoid robots to perform tasks based on human instructions',
    difficulty: 'advanced',
    duration: '4-5 weeks',
    progress: 0,
    completed: false,
    lessons: 16,
    exercises: 8
  }
];

const DIFFICULTY_COLORS = {
  beginner: { bg: '#dcfce7', text: '#166534' },
  intermediate: { bg: '#fef3c7', text: '#92400e' },
  advanced: { bg: '#fecaca', text: '#b91c1c' }
};

export default function ModuleProgress() {
  const [expandedModule, setExpandedModule] = useState(null);

  const toggleModule = (moduleId) => {
    setExpandedModule(expandedModule === moduleId ? null : moduleId);
  };

  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          {MODULES.map((module) => (
            <div key={module.id} className="col col--6 margin-bottom--lg">
              <div className={styles.moduleCard}>
                <div className={styles.moduleHeader}>
                  <span className={styles.moduleIcon}>{module.icon}</span>
                  <h3 className={styles.moduleTitle}>{module.title}</h3>
                  <span
                    className={clsx(styles.difficulty, styles[module.difficulty])}
                    style={{
                      backgroundColor: DIFFICULTY_COLORS[module.difficulty]?.bg,
                      color: DIFFICULTY_COLORS[module.difficulty]?.text
                    }}
                  >
                    {module.difficulty}
                  </span>
                </div>

                <p className={styles.moduleDescription}>{module.description}</p>

                <div className={styles.moduleMeta}>
                  <span className={styles.duration}>⏱️ {module.duration}</span>
                  <span className={styles.lessons}>{module.lessons} lessons</span>
                  <span className={styles.exercises}>{module.exercises} exercises</span>
                </div>

                <div className={styles.moduleProgressContainer}>
                  <div className={styles.progressBar}>
                    <div
                      className={styles.progressFill}
                      style={{ width: `${module.progress}%` }}
                    ></div>
                  </div>
                  <div className={styles.progressText}>{module.progress}% complete</div>
                </div>

                <div className={styles.moduleActions}>
                  <button className="button button--primary">
                    {module.progress > 0 ? 'Continue' : 'Start Module'}
                  </button>
                  <button
                    className="button button--outline button--secondary"
                    onClick={() => toggleModule(module.id)}
                  >
                    {expandedModule === module.id ? 'Show Less' : 'Details'}
                  </button>
                </div>

                {expandedModule === module.id && (
                  <div className={styles.moduleDetails}>
                    <h4>Module Content</h4>
                    <ul>
                      <li>Introduction to core concepts</li>
                      <li>Hands-on exercises</li>
                      <li>Simulation projects</li>
                      <li>Knowledge checks</li>
                    </ul>
                    <h4>Learning Outcomes</h4>
                    <ul>
                      <li>Understand fundamental concepts</li>
                      <li>Implement practical solutions</li>
                      <li>Validate in simulation environment</li>
                    </ul>
                  </div>
                )}
              </div>
            </div>
          ))}
        </div>

        {/* Module Roadmap */}
        <div className={styles.moduleRoadmap}>
          <h3>Learning Pathway</h3>
          <div className={styles.roadmap}>
            {MODULES.map((module, index) => (
              <div key={module.id} className={styles.roadmapItem}>
                <div
                  className={styles.roadmapIcon}
                  style={{
                    backgroundColor: DIFFICULTY_COLORS[module.difficulty]?.bg,
                    color: DIFFICULTY_COLORS[module.difficulty]?.text
                  }}
                >
                  {index + 1}
                </div>
                <div className={styles.roadmapTitle}>{module.title}</div>
                <div className={clsx(styles.roadmapStatus, {
                  [styles.completed]: module.progress === 100,
                  [styles.current]: module.progress > 0 && module.progress < 100,
                  [styles.upcoming]: module.progress === 0
                })}>
                  {module.progress === 100 ? 'Completed' :
                   module.progress > 0 ? 'In Progress' : 'Upcoming'}
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}