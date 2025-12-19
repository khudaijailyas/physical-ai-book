import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: '🚀 Your Next Skill Awaits',
   Svg: require('@site/static/img/undraw_learning.svg').default,
   description: (
      <>
       Learn step by step with clear explanations, practical examples,
        and a beginner-friendly approach designed to build real skills.
      </>
    ),
  },
  {
  title: '🎯 Focus on What Matters',
  Svg: require('@site/static/img/undraw_focus.svg').default,
  description: (
    <>
      No distractions, no unnecessary complexity.
      Focus only on learning the skills that truly matter,
      with a clear and structured path.
    </>
  ),
},

 {
  title: '✨ Learn by Doing',
  Svg: require('@site/static/img/undraw_code_thinking.svg').default,
  description: (
    <>
      Understand concepts through real examples and hands-on practice.
      Learn not just theory, but how things actually work.
    </>
  ),
},

];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
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
