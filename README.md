# Rival Dashboard for ARCh 2026.  

## Getting Started.  

### 1. Clone this repo:

1. Using bash or other terminal

   ```
   git clone https://github.com/yosuahres/rover-dashboard.git
   ```

### 2. Install Dependencies
It is encouraged to use **pnpm** so the packages can work properly.

```bash
pnpm install
```

### 3. Run Development Server (Web)
You can start the web server using this command:
```bash
pnpm dev
```

Open [http://localhost:6969](http://localhost:6969) with your browser to see the result.

### 4. Desktop Application (Tauri)

This project can also be run as a native desktop application using Tauri.

#### Prerequisites for Tauri Development:
Before running or building the desktop application, ensure you have the necessary Tauri development prerequisites installed:
*   **Rust:** Follow the instructions on [https://www.rust-lang.org/tools/install](https://www.rust-lang.org/tools/install).
*   **System Dependencies:**
    *   **macOS:** Run `xcode-select --install` in your terminal.
    *   **Windows:** Install Microsoft Visual Studio C++ Build Tools and WebView2. Refer to the [Tauri Prerequisites Guide](https://tauri.app/v1/guides/getting-started/prerequisites/) for detailed instructions.
    *   **Linux:** Install `webkit2gtk` and other development libraries specific to your distribution. Refer to the [Tauri Prerequisites Guide](https://tauri.app/v1/guides/getting-started/prerequisites/) for detailed instructions.

#### Run Desktop Development Build:
To run the desktop application in development mode:
```bash
pnpm tauri dev
```

#### Build Desktop Application:
To build the production-ready desktop installer:
```bash
pnpm tauri build
```
The installer files will be located in `src-tauri/target/release/bundle/`.

### 5. Commit message
This project uses the conventional commit specification for better readability and clarity. It is mandatory to use conventional commit messages. Read more about conventional commits [here](https://www.conventionalcommits.org/en/v1.0.0/).

## ⁉️ author?   

king hares.
