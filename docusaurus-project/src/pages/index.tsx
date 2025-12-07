import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroSection}>
      <div className={styles.heroContainer}>
        <h1 className={styles.heroTitle}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className={styles.heroSubtitle}>
          Future of Work — Agents, Humans & Robots Together
        </p>

        <div className={styles.heroButtons}>
          <Link className="button button--secondary button--lg" to="/docs/intro">
            Start Learning 
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Learn how the future of work combines AI, humans, and robots.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
