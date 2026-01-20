import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Custom components
import ModuleProgress from '../components/ModuleProgress/ModuleProgress';
import InteractiveCode from '../components/InteractiveCode/InteractiveCode';
import ExerciseCard from '../components/ExerciseCard/ExerciseCard';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>

        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1/intro"
          >
            Start Learning Physical AI
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description="Physical AI Book — learn to build AI systems that perceive, reason, and act in the physical world."
    >
      <HomepageHeader />

      <main>
        <section className={styles.features}>
          <div className="container">

            {/* Modules */}
            <div className="row">
              <div className="col col--4">
                <h2>Module 1: ROS 2 Fundamentals</h2>
                <p>
                  Learn the middleware layer that allows AI software
                  to control robots in simulated and real environments.
                </p>
              </div>

              <div className="col col--4">
                <h2>Module 2: Digital Twin</h2>
                <p>
                  Create digital twins of robots and environments to
                  validate physics, control logic, and interactions.
                </p>
              </div>

              <div className="col col--4">
                <h2>Module 3: AI-Robot Brain</h2>
                <p>
                  Build the AI brain using NVIDIA Isaac Sim and Isaac ROS
                  with photorealistic simulation.
                </p>
              </div>
            </div>

            {/* Progress */}
            <div className="row margin-top--lg">
              <div className="col col--12">
                <h2>Learning Progress</h2>
                <ModuleProgress />
              </div>
            </div>

            {/* Interactive Code */}
            <div className="row margin-top--lg">
              <div className="col col--12">
                <h2>Interactive Learning</h2>

                <InteractiveCode
                  language="python"
                  title="ROS 2 Publisher Example"
                >
{`import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1`}
                </InteractiveCode>
              </div>
            </div>

            {/* Exercises */}
            <div className="row margin-top--lg">
              <div className="col col--12">
                <h2>Hands-on Exercises</h2>

                <ExerciseCard
                  title="Basic Robot Movement"
                  difficulty="Beginner"
                  duration="30 mins"
                  type="exercise"
                  onStart={() => console.log('Starting exercise')}
                >
                  <p>
                    Implement a ROS 2 node that moves a robot forward
                    for five seconds.
                  </p>
                </ExerciseCard>

                <ExerciseCard
                  title="Object Detection Challenge"
                  difficulty="Intermediate"
                  duration="2 hours"
                  type="challenge"
                  onStart={() => console.log('Starting challenge')}
                >
                  <p>
                    Build an object detection pipeline in a Gazebo
                    simulation environment.
                  </p>
                </ExerciseCard>
              </div>
            </div>

            {/* Help */}
            <div className="row margin-top--lg">
              <div className="col col--12">
                <h2>Need Help?</h2>
                <p>
                  Ask our AI tutor about any Physical AI concept using
                  the chat icon in the bottom-right corner.
                </p>
              </div>
            </div>

          </div>
        </section>
      </main>
    </Layout>
  );
}
