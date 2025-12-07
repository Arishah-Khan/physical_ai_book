import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import FeatureCard from '../FeatureCard'; // Import the new FeatureCard component
import styles from '../../css/custom.css'; // Import global styles

type FeatureItem = {
  title: string;
  icon: React.ComponentType<React.ComponentProps<'svg'>>; // Changed Svg to icon
  description: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Introduction to Humanoid Robotics',
    icon: require('@site/static/img/robot.svg').default, // Using a generic robot icon for now
    description: 'Learn the fundamentals of humanoid robots, from design to real-world applications.',
  },
  {
    title: 'Physical AI Systems',
    icon: require('@site/static/img/undraw_docusaurus_react.svg').default, // Using a generic AI icon for now
    description: 'Understand how AI agents interact with the physical world using sensors and actuators.',
  },
  {
    title: 'Future of Human-Agent Collaboration',
    icon: require('@site/static/img/undraw_docusaurus_tree.svg').default, // Using a generic collaboration icon for now
    description: 'Explore how humans and intelligent agents can work together to solve complex problems.',
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
<section
  className={clsx('features', 'padding-vert--md', 'hero--dark')}
  style={{ backgroundColor: 'var(--ifm-background-color)' }}
>
      <div className="container">
        <div className="row">
          {FeatureList.map((feature, idx) => (
            <FeatureCard key={idx} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}
