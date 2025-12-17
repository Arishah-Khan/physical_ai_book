---
description: Swizzle Docusaurus components to inject custom widgets into all documentation pages
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Overview

Generate everything needed to inject custom React components into Docusaurus documentation pages by:
1. Swizzling the appropriate Docusaurus component
2. Creating a wrapper that preserves existing functionality
3. Injecting custom widgets at specified locations
4. Adding configuration support
5. Maintaining TypeScript compatibility
6. Supporting multiple injection points

## Implementation Steps

### 1. Analyze User Requirements

Extract from user input (if provided):
- Component to inject (default: ChatWidget)
- Injection point (default: DocItem/Layout/Footer)
- Configuration options
- Whether to use TypeScript
- Custom props to pass
- Conditional rendering rules

### 2. Check Docusaurus Version and Setup

```bash
# Check if it's a Docusaurus project
if [ -f "docusaurus.config.js" ] || [ -f "docusaurus.config.ts" ]; then
  echo "Docusaurus project detected"
else
  echo "ERROR: Not a Docusaurus project"
  exit 1
fi

# Check Docusaurus version
npm list @docusaurus/core
```

### 3. Swizzle DocItem Component (Recommended Approach)

The DocItem/Layout component is the best injection point for page-level widgets.

#### Step 1: Swizzle the component

```bash
# For JavaScript projects
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap --typescript

# For TypeScript projects (recommended)
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap --typescript
```

This creates: `src/theme/DocItem/Layout/index.tsx` (or `.js`)

#### Step 2: Generate the wrapper component

```tsx
// src/theme/DocItem/Layout/index.tsx

import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';

// Import your custom widget(s)
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';
// Add more widgets as needed
// import MyCustomWidget from '@site/src/components/MyCustomWidget';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      {/* Render original DocItem Layout */}
      <Layout {...props} />

      {/* Inject your custom widget(s) */}
      <ChatWidget
        apiEndpoint="/api/chat/query"
        primaryColor="#25c2a0"
        position="bottom-right"
        title="Documentation Assistant"
        enableSelection={true}
      />

      {/* Add more widgets here */}
      {/* <MyCustomWidget /> */}
    </>
  );
}
```

### 4. Alternative Injection Points

Depending on where you want the widget to appear:

#### Option A: Root Layout (Site-wide)

Swizzle the Root component for site-wide injection:

```bash
npm run swizzle @docusaurus/theme-classic Root -- --wrap
```

```tsx
// src/theme/Root.tsx

import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget
        apiEndpoint="/api/chat/query"
        position="bottom-right"
      />
    </>
  );
}
```

#### Option B: Layout (Wrapper for all pages)

```bash
npm run swizzle @docusaurus/theme-classic Layout -- --wrap
```

```tsx
// src/theme/Layout/index.tsx

import React from 'react';
import Layout from '@theme-original/Layout';
import type { WrapperProps } from '@docusaurus/types';
import type LayoutType from '@theme/Layout';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <Layout {...props} />
      <ChatWidget />
    </>
  );
}
```

#### Option C: Footer (Bottom of page)

```bash
npm run swizzle @docusaurus/theme-classic Footer -- --wrap
```

```tsx
// src/theme/Footer/index.tsx

import React from 'react';
import Footer from '@theme-original/Footer';
import type FooterType from '@theme/Footer';
import type { WrapperProps } from '@docusaurus/types';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

type Props = WrapperProps<typeof FooterType>;

export default function FooterWrapper(props: Props): JSX.Element {
  return (
    <>
      <Footer {...props} />
      <ChatWidget position="bottom-right" />
    </>
  );
}
```

### 5. Configuration-Based Injection

Create a configuration file for dynamic widget management:

```typescript
// src/config/widgetConfig.ts

export interface WidgetConfig {
  enabled: boolean;
  component?: string;
  props?: Record<string, any>;
  conditions?: {
    paths?: string[];      // Only show on these paths
    excludePaths?: string[]; // Don't show on these paths
    docOnly?: boolean;     // Only on doc pages
    blogOnly?: boolean;    // Only on blog pages
  };
}

export interface WidgetsConfig {
  chatWidget?: WidgetConfig;
  feedbackWidget?: WidgetConfig;
  [key: string]: WidgetConfig | undefined;
}

const widgetsConfig: WidgetsConfig = {
  chatWidget: {
    enabled: true,
    props: {
      apiEndpoint: '/api/chat/query',
      primaryColor: '#25c2a0',
      position: 'bottom-right',
      title: 'Documentation Assistant',
      enableSelection: true,
    },
    conditions: {
      // Only show on doc pages
      docOnly: true,
      // Exclude certain paths
      excludePaths: ['/search'],
    },
  },
  feedbackWidget: {
    enabled: false,
    props: {
      position: 'bottom-left',
    },
  },
};

export default widgetsConfig;
```

### 6. Smart Wrapper with Configuration

```tsx
// src/theme/DocItem/Layout/index.tsx

import React from 'react';
import { useLocation } from '@docusaurus/router';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';
import WidgetInjector from '@site/src/components/WidgetInjector';
import widgetsConfig from '@site/src/config/widgetConfig';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  const location = useLocation();

  return (
    <>
      <Layout {...props} />
      <WidgetInjector
        location={location}
        config={widgetsConfig}
        pageType="doc"
      />
    </>
  );
}
```

### 7. Widget Injector Component

```tsx
// src/components/WidgetInjector/index.tsx

import React from 'react';
import type { Location } from '@docusaurus/router';
import type { WidgetsConfig, WidgetConfig } from '@site/src/config/widgetConfig';

// Import all available widgets
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';
// Add more widget imports as needed

interface WidgetInjectorProps {
  location: Location;
  config: WidgetsConfig;
  pageType?: 'doc' | 'blog' | 'page';
}

// Map of available widgets
const WIDGET_COMPONENTS = {
  chatWidget: ChatWidget,
  // Add more widgets here
  // feedbackWidget: FeedbackWidget,
};

function shouldRenderWidget(
  config: WidgetConfig,
  location: Location,
  pageType?: string
): boolean {
  if (!config.enabled) {
    return false;
  }

  const { conditions } = config;
  if (!conditions) {
    return true;
  }

  // Check page type conditions
  if (conditions.docOnly && pageType !== 'doc') {
    return false;
  }
  if (conditions.blogOnly && pageType !== 'blog') {
    return false;
  }

  // Check path conditions
  const currentPath = location.pathname;

  if (conditions.paths && conditions.paths.length > 0) {
    const isIncluded = conditions.paths.some(path =>
      currentPath.startsWith(path)
    );
    if (!isIncluded) {
      return false;
    }
  }

  if (conditions.excludePaths && conditions.excludePaths.length > 0) {
    const isExcluded = conditions.excludePaths.some(path =>
      currentPath.startsWith(path)
    );
    if (isExcluded) {
      return false;
    }
  }

  return true;
}

const WidgetInjector: React.FC<WidgetInjectorProps> = ({
  location,
  config,
  pageType,
}) => {
  return (
    <>
      {Object.entries(config).map(([widgetKey, widgetConfig]) => {
        if (!widgetConfig || !shouldRenderWidget(widgetConfig, location, pageType)) {
          return null;
        }

        const WidgetComponent = WIDGET_COMPONENTS[widgetKey];
        if (!WidgetComponent) {
          console.warn(`Widget component not found: ${widgetKey}`);
          return null;
        }

        return (
          <WidgetComponent
            key={widgetKey}
            {...(widgetConfig.props || {})}
          />
        );
      })}
    </>
  );
};

export default WidgetInjector;
```

### 8. Environment-Based Configuration

Add support for environment-specific settings:

```typescript
// src/config/widgetConfig.ts (enhanced version)

const isDev = process.env.NODE_ENV === 'development';
const isProd = process.env.NODE_ENV === 'production';

export interface WidgetConfig {
  enabled: boolean;
  component?: string;
  props?: Record<string, any>;
  conditions?: {
    paths?: string[];
    excludePaths?: string[];
    docOnly?: boolean;
    blogOnly?: boolean;
    environments?: ('development' | 'production')[];
  };
}

export interface WidgetsConfig {
  chatWidget?: WidgetConfig;
  [key: string]: WidgetConfig | undefined;
}

const widgetsConfig: WidgetsConfig = {
  chatWidget: {
    enabled: true,
    props: {
      apiEndpoint: isProd
        ? 'https://api.example.com/chat/query'
        : 'http://localhost:8000/api/chat/query',
      primaryColor: '#25c2a0',
      position: 'bottom-right',
      title: 'Documentation Assistant',
      enableSelection: true,
      maxMessages: 50,
    },
    conditions: {
      docOnly: true,
      excludePaths: ['/search', '/404'],
      // Only show in production, or explicitly enable in dev
      environments: process.env.ENABLE_CHAT_DEV === 'true'
        ? ['development', 'production']
        : ['production'],
    },
  },
};

export default widgetsConfig;
```

### 9. Enhanced Widget Injector with Environment Check

```tsx
// src/components/WidgetInjector/index.tsx (enhanced)

import React from 'react';
import type { Location } from '@docusaurus/router';
import type { WidgetsConfig, WidgetConfig } from '@site/src/config/widgetConfig';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

interface WidgetInjectorProps {
  location: Location;
  config: WidgetsConfig;
  pageType?: 'doc' | 'blog' | 'page';
}

const WIDGET_COMPONENTS = {
  chatWidget: ChatWidget,
};

function shouldRenderWidget(
  config: WidgetConfig,
  location: Location,
  pageType?: string
): boolean {
  if (!config.enabled) {
    return false;
  }

  const { conditions } = config;
  if (!conditions) {
    return true;
  }

  // Check environment
  if (conditions.environments && conditions.environments.length > 0) {
    const currentEnv = process.env.NODE_ENV;
    if (!conditions.environments.includes(currentEnv as any)) {
      return false;
    }
  }

  // Check page type
  if (conditions.docOnly && pageType !== 'doc') {
    return false;
  }
  if (conditions.blogOnly && pageType !== 'blog') {
    return false;
  }

  // Check paths
  const currentPath = location.pathname;

  if (conditions.paths && conditions.paths.length > 0) {
    const isIncluded = conditions.paths.some(path =>
      currentPath.startsWith(path)
    );
    if (!isIncluded) {
      return false;
    }
  }

  if (conditions.excludePaths && conditions.excludePaths.length > 0) {
    const isExcluded = conditions.excludePaths.some(path =>
      currentPath.startsWith(path)
    );
    if (isExcluded) {
      return false;
    }
  }

  return true;
}

const WidgetInjector: React.FC<WidgetInjectorProps> = ({
  location,
  config,
  pageType,
}) => {
  const [mounted, setMounted] = React.useState(false);

  // Ensure client-side only rendering for widgets
  React.useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return null;
  }

  return (
    <>
      {Object.entries(config).map(([widgetKey, widgetConfig]) => {
        if (!widgetConfig || !shouldRenderWidget(widgetConfig, location, pageType)) {
          return null;
        }

        const WidgetComponent = WIDGET_COMPONENTS[widgetKey];
        if (!WidgetComponent) {
          if (process.env.NODE_ENV === 'development') {
            console.warn(`Widget component not found: ${widgetKey}`);
          }
          return null;
        }

        return (
          <WidgetComponent
            key={widgetKey}
            {...(widgetConfig.props || {})}
          />
        );
      })}
    </>
  );
};

export default WidgetInjector;
```

### 10. Docusaurus Config Integration

Update `docusaurus.config.js` to pass configuration:

```javascript
// docusaurus.config.js

module.exports = {
  // ... other config

  customFields: {
    widgets: {
      chatWidget: {
        enabled: true,
        apiEndpoint: process.env.CHAT_API_ENDPOINT || '/api/chat/query',
      },
    },
  },

  // ... rest of config
};
```

Access in components:

```tsx
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function MyComponent() {
  const { siteConfig } = useDocusaurusContext();
  const widgetConfig = siteConfig.customFields?.widgets?.chatWidget;

  // Use widgetConfig
}
```

### 11. Complete Setup Script

```bash
#!/bin/bash
# setup-widget-injection.sh

echo "Setting up Docusaurus Widget Injection..."

# Step 1: Check if Docusaurus project
if [ ! -f "docusaurus.config.js" ] && [ ! -f "docusaurus.config.ts" ]; then
  echo "ERROR: Not a Docusaurus project"
  exit 1
fi

# Step 2: Create directories
echo "Creating directories..."
mkdir -p src/components/WidgetInjector
mkdir -p src/config
mkdir -p src/theme/DocItem/Layout

# Step 3: Swizzle DocItem Layout
echo "Swizzling DocItem/Layout component..."
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap --typescript --danger

# Step 4: Copy generated files
echo "Files created. Please review:"
echo "  - src/theme/DocItem/Layout/index.tsx"
echo "  - src/components/WidgetInjector/index.tsx"
echo "  - src/config/widgetConfig.ts"

echo ""
echo "Next steps:"
echo "1. Add your widget component to src/components/"
echo "2. Update src/config/widgetConfig.ts with your widget settings"
echo "3. Register widget in WidgetInjector WIDGET_COMPONENTS map"
echo "4. Test by running 'npm start'"
```

### 12. TypeScript Configuration

Ensure proper TypeScript setup:

```json
// tsconfig.json (add if not present)

{
  "extends": "@docusaurus/tsconfig",
  "compilerOptions": {
    "baseUrl": ".",
    "paths": {
      "@site/*": ["src/*"],
      "@theme/*": ["src/theme/*"],
      "@theme-original/*": [
        "@docusaurus/theme-classic/src/theme/*"
      ]
    }
  },
  "include": ["src/**/*"]
}
```

### 13. Testing the Injection

Create a test page to verify widget injection:

```tsx
// src/pages/test-widget.tsx

import React from 'react';
import Layout from '@theme/Layout';

export default function TestWidget(): JSX.Element {
  return (
    <Layout
      title="Widget Test Page"
      description="Test page for widget injection"
    >
      <div style={{ padding: '2rem' }}>
        <h1>Widget Test Page</h1>
        <p>
          This page is used to test widget injection.
          The chat widget should appear in the bottom-right corner.
        </p>

        <div style={{ marginTop: '2rem' }}>
          <h2>Test Text Selection</h2>
          <p>
            Select this text to test the selection mode feature.
            The widget should detect the selection and switch to selection mode.
            You can then ask questions about the selected text.
          </p>
        </div>

        <div style={{ marginTop: '2rem', background: '#f5f5f5', padding: '1rem' }}>
          <h3>Sample Content</h3>
          <p>
            Lorem ipsum dolor sit amet, consectetur adipiscing elit.
            Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
            Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris.
          </p>
        </div>
      </div>
    </Layout>
  );
}
```

### 14. Debugging Tools

Add debugging utilities:

```tsx
// src/components/WidgetInjector/debug.tsx

import React from 'react';
import type { Location } from '@docusaurus/router';
import type { WidgetsConfig } from '@site/src/config/widgetConfig';

interface DebugPanelProps {
  location: Location;
  config: WidgetsConfig;
  pageType?: string;
}

export const DebugPanel: React.FC<DebugPanelProps> = ({
  location,
  config,
  pageType,
}) => {
  const [isOpen, setIsOpen] = React.useState(false);

  if (process.env.NODE_ENV !== 'development') {
    return null;
  }

  return (
    <div style={{
      position: 'fixed',
      top: 10,
      left: 10,
      zIndex: 10000,
      background: 'rgba(0,0,0,0.8)',
      color: 'white',
      padding: '10px',
      borderRadius: '8px',
      fontSize: '12px',
      maxWidth: '300px',
    }}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        style={{
          background: 'white',
          color: 'black',
          border: 'none',
          padding: '5px 10px',
          borderRadius: '4px',
          cursor: 'pointer',
        }}
      >
        {isOpen ? 'Hide' : 'Show'} Widget Debug
      </button>

      {isOpen && (
        <div style={{ marginTop: '10px' }}>
          <div><strong>Page Type:</strong> {pageType}</div>
          <div><strong>Path:</strong> {location.pathname}</div>
          <div><strong>Widgets:</strong></div>
          <ul style={{ margin: '5px 0', paddingLeft: '20px' }}>
            {Object.entries(config).map(([key, cfg]) => (
              <li key={key}>
                {key}: {cfg?.enabled ? '✅' : '❌'}
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};
```

Add to WidgetInjector:

```tsx
import { DebugPanel } from './debug';

// Inside component:
return (
  <>
    {process.env.NODE_ENV === 'development' && (
      <DebugPanel location={location} config={config} pageType={pageType} />
    )}
    {/* ... rest of widgets */}
  </>
);
```

### 15. Documentation

Create README for the widget injection system:

```markdown
# Widget Injection System

This Docusaurus site uses a custom widget injection system that allows you to add React components to all pages.

## Architecture

- **Swizzled Component**: `src/theme/DocItem/Layout/index.tsx`
- **Widget Injector**: `src/components/WidgetInjector/index.tsx`
- **Configuration**: `src/config/widgetConfig.ts`

## Adding a New Widget

1. Create your widget component in `src/components/YourWidget/`
2. Import it in `src/components/WidgetInjector/index.tsx`:
   ```tsx
   import YourWidget from '@site/src/components/YourWidget';
   ```
3. Add to `WIDGET_COMPONENTS` map:
   ```tsx
   const WIDGET_COMPONENTS = {
     chatWidget: ChatWidget,
     yourWidget: YourWidget,  // Add here
   };
   ```
4. Configure in `src/config/widgetConfig.ts`:
   ```tsx
   const widgetsConfig: WidgetsConfig = {
     yourWidget: {
       enabled: true,
       props: {
         // Your widget props
       },
       conditions: {
         // Optional rendering conditions
       },
     },
   };
   ```

## Configuration Options

### Widget Config Structure

```typescript
{
  enabled: boolean;              // Enable/disable widget
  props?: Record<string, any>;   // Props to pass to widget
  conditions?: {
    paths?: string[];            // Only show on these paths
    excludePaths?: string[];     // Don't show on these paths
    docOnly?: boolean;           // Only on doc pages
    blogOnly?: boolean;          // Only on blog pages
    environments?: string[];     // Only in these environments
  };
}
```

## Conditional Rendering

### Show only on specific pages
```typescript
conditions: {
  paths: ['/docs/', '/api/'],
}
```

### Exclude from pages
```typescript
conditions: {
  excludePaths: ['/search', '/404'],
}
```

### Environment-specific
```typescript
conditions: {
  environments: ['production'],
}
```

## Debugging

In development mode, a debug panel appears in the top-left corner showing:
- Current page type
- Current path
- Widget enable/disable status

## Testing

Test your widget injection on: `/test-widget`
```

## Deliverables

When complete, provide:

1. **Swizzled component** - `src/theme/DocItem/Layout/index.tsx`
2. **WidgetInjector** - Smart component loader with conditions
3. **Configuration file** - `widgetConfig.ts` with examples
4. **TypeScript definitions** - Full type safety
5. **Setup script** - Automated swizzling and file creation
6. **Debug tools** - Development-mode debugging panel
7. **Test page** - Widget testing playground
8. **Documentation** - Complete usage guide

## Features

- ✅ Preserves all Docusaurus functionality
- ✅ TypeScript support
- ✅ Configuration-based widget management
- ✅ Conditional rendering (paths, page types, environments)
- ✅ Multiple injection points supported
- ✅ Client-side only rendering (SSR safe)
- ✅ Debug mode for development
- ✅ Environment-specific settings
- ✅ Easy to add new widgets
- ✅ No modification to Docusaurus core
- ✅ Full theme compatibility

## Supported Injection Points

1. **DocItem/Layout** - Documentation pages (recommended)
2. **Root** - All pages site-wide
3. **Layout** - All pages with layout wrapper
4. **Footer** - Bottom of all pages
5. **Navbar** - Top of all pages (advanced)

## Notes

- Always use `--wrap` flag when swizzling to preserve functionality
- Test thoroughly after swizzling to ensure no regressions
- Use TypeScript for better type safety
- Keep widget logic separate from injection logic
- Use configuration files for easy widget management
- Consider performance impact of multiple widgets
