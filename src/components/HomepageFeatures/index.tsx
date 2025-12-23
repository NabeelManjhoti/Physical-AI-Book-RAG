import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Embodied Intelligence',
    description: (
      <>
        Learn how AI systems can be embodied in robotic platforms to create
        intelligent agents that interact with the physical world through
        perception, reasoning, and action.
      </>
    ),
  },
  {
    title: 'Modern Robotics Stack',
    description: (
      <>
        Master the complete robotics development stack: ROS 2 for communication,
        Gazebo/Unity for simulation, NVIDIA Isaac for GPU-accelerated perception,
        and VLA models for intelligent planning.
      </>
    ),
  },
  {
    title: 'Practical Applications',
    description: (
      <>
        Build humanoid robots that can understand natural language commands,
        navigate complex environments, manipulate objects, and operate safely
        in dynamic real-world scenarios.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureIcon}>
          <svg width="64" height="64" viewBox="0 0 100 100" fill="none" xmlns="http://www.w3.org/2000/svg">
            <rect width="100" height="100" fill="url(#gradient)" opacity="0.1"/>
            <path d="M30,30 L70,30 L60,50 L70,70 L30,70 L40,50 Z" fill="var(--ifm-color-primary)" stroke="var(--ifm-color-primary-light)" strokeWidth="2"/>
            <circle cx="50" cy="50" r="15" fill="none" stroke="var(--ifm-color-primary-light)" strokeWidth="2" strokeDasharray="3,3"/>
            <defs>
              <linearGradient id="gradient" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="var(--ifm-color-primary)"/>
                <stop offset="100%" stopColor="var(--ifm-color-primary-darker)"/>
              </linearGradient>
            </defs>
          </svg>
        </div>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
