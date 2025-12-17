import React from 'react';
import clsx from 'clsx';
import styles from './Diagram.module.css';

// Component for ROS 2 diagrams and architecture illustrations
export default function ROS2Diagram({title, description, children, className}) {
  return (
    <div className={clsx('margin-vert--md', className)}>
      <div className={styles.diagramContainer}>
        <h3>{title}</h3>
        <div className={styles.diagramContent}>
          {children}
        </div>
        <p className={styles.diagramDescription}>{description}</p>
      </div>
    </div>
  );
}