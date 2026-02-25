// Default theme = dark
if (!localStorage.getItem("theme")) {
    localStorage.setItem("theme", "dark");
}

function applyTheme() {
    const t = localStorage.getItem("theme");

    if (t === "light") {
        document.documentElement.style.setProperty("--bg", "#f5f5f5");
        document.documentElement.style.setProperty("--fg", "#111");
        document.documentElement.style.setProperty("--fg-soft", "#444");
        document.documentElement.style.setProperty("--fg-strong", "#000");
        document.documentElement.style.setProperty("--card", "#fff");
        document.documentElement.style.setProperty("--border", "#ccc");
        document.documentElement.style.setProperty("--accent", "#0077cc");
        document.documentElement.style.setProperty("--accent-glow", "#66bfff");
    } else {
        document.documentElement.style.setProperty("--bg", "#0a0a0f");
        document.documentElement.style.setProperty("--fg", "#e0e0ff");
        document.documentElement.style.setProperty("--fg-soft", "#8aa");
        document.documentElement.style.setProperty("--fg-strong", "#fff");
        document.documentElement.style.setProperty("--card", "#11111a");
        document.documentElement.style.setProperty("--border", "#222");
        document.documentElement.style.setProperty("--accent", "#6ecbff");
        document.documentElement.style.setProperty("--accent-glow", "#0099ff");
    }
}

applyTheme();

// Toggle theme
function toggleTheme() {
    const t = localStorage.getItem("theme");
    localStorage.setItem("theme", t === "light" ? "dark" : "light");
    applyTheme();
}
