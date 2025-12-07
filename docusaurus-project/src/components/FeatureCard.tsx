import React from 'react';
import clsx from 'clsx';
import styles from './FeatureCard.module.css';

type FeatureCardProps = {
  title: string;
  icon: React.ComponentType<React.ComponentProps<'svg'>>;
  description: string;
};

export default function FeatureCard({ title, icon: Icon, description }: FeatureCardProps){
  return (
    <div className={clsx('col col--4 margin-bottom--lg', styles.featureCard)}>
      <div className={clsx('card', styles.card)}>
        <div className={clsx('card__header', styles.cardHeader)}>
          <Icon className={styles.featureIcon} />
          <h3 className={styles.featureTitle}>{title}</h3>
        </div>
        <div className={clsx('card__body', styles.cardBody)}>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}