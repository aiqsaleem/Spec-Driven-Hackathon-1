import React, { JSX } from 'react';
import clsx from 'clsx';
import styles from './RoboticsCard.module.css';

type FeatureItem = {
  title: string;
  description: JSX.Element;
};

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function RoboticsCard(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <Feature
            title="ROS 2 Fundamentals"
            description={
              <>
                Learn the core concepts of ROS 2 including nodes, topics, services, and actions.
              </>
            }
          />
          <Feature
            title="Python Integration"
            description={
              <>
                Bridge AI agents written in Python to ROS controllers using rclpy.
              </>
            }
          />
          <Feature
            title="Robot Modeling"
            description={
              <>
                Model robot bodies using URDF for accurate simulation and control.
              </>
            }
          />
        </div>
      </div>
    </section>
  );
}