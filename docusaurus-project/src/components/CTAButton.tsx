import React from 'react';
import clsx from 'clsx';
import styles from './CTAButton.module.css';

type CTAButtonProps = {
  label: string;
  to: string;
  type?: 'primary' | 'secondary';
};

export default function CTAButton({ label, to, type = 'primary' }: CTAButtonProps){
  return (
    <a className={clsx('button button--lg', styles.ctaButton, styles[type])} href={to}>
      {label}
    </a>
  );
}