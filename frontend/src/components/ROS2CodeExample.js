import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import CodeBlock from '@theme/CodeBlock';

// Component for ROS 2 code examples with multiple language tabs
export default function ROS2CodeExample({children, title, languages}) {
  return (
    <div className="text--center padding-horiz--md">
      <h3>{title}</h3>
      <Tabs groupId="programming-language">
        {languages.map((lang) => (
          <TabItem value={lang.value} label={lang.label} key={lang.value}>
            <CodeBlock language={lang.language}>
              {lang.code}
            </CodeBlock>
          </TabItem>
        ))}
      </Tabs>
    </div>
  );
}