import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    
<header className={clsx('hero hero--primary', styles.heroBanner)}>



      <div className="container">
        <Heading as="h1" className="hero__title">
          🚀 Your Next Skill Awaits
        </Heading>

        <p className="hero__subtitle">
          A simple, clear, and beginner-friendly book to help you learn
          step by step — without confusion or overload.
        </p>
        


        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/physical-ai-introduction">
            ✨ Begin Your Learning Journey
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
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
