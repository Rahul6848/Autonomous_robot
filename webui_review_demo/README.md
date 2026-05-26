# Web UI Review Demo

This folder contains a standalone static review build of the robot dashboard.

It is intentionally isolated from the live ROS 2 dashboard:

- no ROS 2 dependency
- no backend API calls
- no robot command publishing
- no stack auto-start
- no camera or desktop streaming requirement

## Files

- `index.html`: standalone review page
- `style.css`: demo styling
- `app.js`: local mock data and simulated interactions

## Local preview

Open `index.html` directly in a browser, or serve the folder with any static server.

Examples:

```bash
cd /home/rahul/ros2_workspace/webui_review_demo
python3 -m http.server 9000
```

Then open:

```text
http://localhost:9000
```

## Public sharing options

### Netlify

1. Zip the `webui_review_demo` folder or drag the folder into Netlify Drop.
2. Netlify will generate a public HTTPS link.
3. Share that link with the company.

### Vercel

1. Import this folder as a static site project.
2. No build command is required.
3. Output directory is the project root.

### GitHub Pages

1. Push `webui_review_demo` to a repository.
2. Enable GitHub Pages from the branch and folder containing `index.html`.
3. Share the generated GitHub Pages URL.

## Purpose

Use this review build when you want to show:

- overall UI design
- layout and information architecture
- command-control flow
- how camera and desktop panels appear
- status and logs presentation

Do not use this folder for live robot control.
