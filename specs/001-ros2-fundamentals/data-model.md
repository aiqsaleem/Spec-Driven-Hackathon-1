# Data Model: ROS 2 Fundamentals Course

## Course Structure

### Module Entity
- **Name**: String (required) - Module identifier (e.g., "Module 1: The Robotic Nervous System")
- **Description**: String (required) - Brief overview of the module's content
- **Learning Objectives**: Array<String> (required) - List of specific learning goals
- **Prerequisites**: Array<String> (optional) - Required knowledge before starting
- **Duration**: Number (optional) - Estimated completion time in minutes
- **Chapters**: Array<Chapter> (required) - Collection of chapters in the module

### Chapter Entity
- **Title**: String (required) - Chapter name
- **Content**: String (required) - Markdown content of the chapter
- **Learning Objectives**: Array<String> (required) - Specific goals for this chapter
- **Examples**: Array<Example> (optional) - Code examples and diagrams
- **Exercises**: Array<Exercise> (optional) - Practice problems for students
- **Duration**: Number (optional) - Estimated time to complete

### Example Entity
- **Title**: String (required) - Brief description of the example
- **Code**: String (required) - The actual code content
- **Language**: String (required) - Programming language (e.g., "python", "xml")
- **Description**: String (required) - Explanation of what the example demonstrates
- **Expected Output**: String (optional) - What the example should produce

### Exercise Entity
- **Title**: String (required) - Exercise name
- **Description**: String (required) - Problem statement for the student
- **Difficulty**: Enum (required) - "beginner", "intermediate", "advanced"
- **Hints**: Array<String> (optional) - Guidance for solving the exercise
- **Solution**: String (optional) - Reference solution

## ROS 2 Domain Entities

### ROS 2 Node Entity
- **Name**: String (required) - Node identifier
- **Description**: String (optional) - Purpose of the node
- **Published Topics**: Array<Topic> (optional) - Topics this node publishes to
- **Subscribed Topics**: Array<Topic> (optional) - Topics this node subscribes to
- **Services**: Array<Service> (optional) - Services this node provides
- **Actions**: Array<Action> (optional) - Actions this node can perform

### Topic Entity
- **Name**: String (required) - Topic identifier
- **Type**: String (required) - Message type (e.g., "std_msgs/String")
- **Description**: String (optional) - Purpose of the topic

### Service Entity
- **Name**: String (required) - Service identifier
- **Type**: String (required) - Service type (e.g., "std_srvs/SetBool")
- **Description**: String (optional) - Purpose of the service

### Action Entity
- **Name**: String (required) - Action identifier
- **Type**: String (required) - Action type (e.g., "example_interfaces/Fibonacci")
- **Description**: String (optional) - Purpose of the action

## Validation Rules

### Module Validation
- Must have at least one chapter
- Learning objectives must be specific and measurable
- Duration must be positive if provided

### Chapter Validation
- Content must be in valid markdown format
- Must have at least one learning objective
- Examples must have corresponding descriptions

### Example Validation
- Code must be syntactically valid in the specified language
- Language must be a supported language type
- Expected output must match the code functionality if provided

## State Transitions

### Module States
- **Draft**: Initial state, content is being created
- **Review**: Content is under review by team members
- **Published**: Content is available to students
- **Archived**: Content is no longer maintained

### Chapter States
- **Draft**: Initial state, content is being created
- **Review**: Content is under review by team members
- **Published**: Content is available to students
- **Archived**: Content is no longer maintained