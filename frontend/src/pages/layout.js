import React from 'react';
import Layout from '@theme/Layout';

// Root layout component that wraps all pages
export default function RootLayout(props) {
  return (
    <Layout {...props}>
      {props.children}
    </Layout>
  );
}