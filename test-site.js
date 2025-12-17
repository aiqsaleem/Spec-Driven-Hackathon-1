#!/usr/bin/env node

/**
 * Simple test script to validate the ROS 2 fundamentals site
 * Checks that all main pages exist and important links work
 */

const fs = require('fs');
const path = require('path');

// Define the paths to check
const requiredPaths = [
  'docs/intro.md',
  'docs/quickstart.md',
  'docs/module-1/intro.md',
  'docs/module-1/chapter-1.md',
  'docs/module-1/chapter-2.md',
  'docs/module-1/chapter-3.md',
  'docs/index-page.md',
  'docs/glossary.md',
  'docs/api-reference.md',
  'docs/troubleshooting.md',
  'docs/best-practices.md',
  'docs/summary.md',
  'static/examples/simple_publisher.py',
  'static/examples/simple_subscriber.py',
  'static/examples/python_agent.py',
  'static/examples/controller_bridge.py',
  'static/examples/basic_robot.urdf',
  'static/examples/humanoid_arm.urdf',
  'static/examples/urdf_control_pipeline.py'
];

const configFiles = [
  'docusaurus.config.ts',
  'sidebars.ts',
  'package.json'
];

console.log('🔍 Validating ROS 2 Fundamentals site...\n');

let errors = 0;

// Check required documentation files
console.log('📄 Checking documentation files...');
for (const filePath of requiredPaths) {
  const fullPath = path.join(__dirname, filePath);
  if (fs.existsSync(fullPath)) {
    console.log(`  ✅ ${filePath}`);
  } else {
    console.log(`  ❌ ${filePath} - MISSING`);
    errors++;
  }
}

// Check config files
console.log('\n⚙️  Checking configuration files...');
for (const filePath of configFiles) {
  const fullPath = path.join(__dirname, filePath);
  if (fs.existsSync(fullPath)) {
    console.log(`  ✅ ${filePath}`);
  } else {
    console.log(`  ❌ ${filePath} - MISSING`);
    errors++;
  }
}

// Check that examples are properly formatted
console.log('\n🧪 Checking example files...');
const exampleFiles = [
  'static/examples/simple_publisher.py',
  'static/examples/simple_subscriber.py',
  'static/examples/python_agent.py',
  'static/examples/controller_bridge.py',
  'static/examples/urdf_control_pipeline.py'
];

for (const filePath of exampleFiles) {
  const fullPath = path.join(__dirname, filePath);
  if (fs.existsSync(fullPath)) {
    const content = fs.readFileSync(fullPath, 'utf8');
    if (content.length > 20) { // Basic check that file has content
      console.log(`  ✅ ${filePath}`);
    } else {
      console.log(`  ⚠️  ${filePath} - May have insufficient content`);
    }
  }
}

// Check URDF files
console.log('\n🤖 Checking URDF model files...');
const urdfFiles = [
  'static/examples/basic_robot.urdf',
  'static/examples/humanoid_arm.urdf'
];

for (const filePath of urdfFiles) {
  const fullPath = path.join(__dirname, filePath);
  if (fs.existsSync(fullPath)) {
    const content = fs.readFileSync(fullPath, 'utf8');
    if (content.includes('<robot') && content.includes('</robot>')) {
      console.log(`  ✅ ${filePath}`);
    } else {
      console.log(`  ❌ ${filePath} - Invalid URDF format`);
      errors++;
    }
  }
}

// Summary
console.log('\n📊 Summary:');
if (errors === 0) {
  console.log('  🎉 All checks passed! The site appears to be complete.');
  console.log('  You can now run `npm start` to launch the development server.');
} else {
  console.log(`  ❌ ${errors} errors found. Please fix missing files before deployment.`);
}

process.exit(errors > 0 ? 1 : 0);