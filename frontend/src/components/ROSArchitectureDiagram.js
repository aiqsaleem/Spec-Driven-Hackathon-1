import React from 'react';
import clsx from 'clsx';
import styles from './Diagram.module.css';

// Component for ROS 2 architecture diagrams using Mermaid syntax
export default function ROSArchitectureDiagram({title, description, mermaidCode}) {
  return (
    <div className={clsx('margin-vert--md')}>
      <div className={styles.diagramContainer}>
        <h3>{title}</h3>
        <div className={styles.diagramContent}>
          <pre className="mermaid">
            {mermaidCode}
          </pre>
        </div>
        <p className={styles.diagramDescription}>{description}</p>
      </div>
    </div>
  );
}