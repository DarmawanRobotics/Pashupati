#!/usr/bin/env python3
"""
Build the PDF guides from their Markdown sources.

    pip install weasyprint markdown pyyaml       # macOS: brew install pango first
    python3 docs/pdf/build.py                    # every guide
    python3 docs/pdf/build.py manual             # one of: manual, tuning

Pages are A5, which reads well on a phone. Text is Barlow Semi Condensed (docs/pdf/fonts, SIL
Open Font License). A source starts with a YAML front matter (title, subtitle, version, date,
revisions) and may contain `{{ parameter_reference }}`, replaced by tables of every parameter in
the packages' config files, read with their comments.
"""

from __future__ import annotations

import html
from pathlib import Path
import re
import sys

import markdown
from params import parameter_reference
from weasyprint import CSS, HTML
from weasyprint.text.fonts import FontConfiguration
import yaml

DOCS = Path(__file__).resolve().parents[1]
ROOT = DOCS.parent
GUIDES = {
    'manual': (DOCS / 'manual' / 'user-manual.md', DOCS / 'Pashupati-User-Manual.pdf'),
    'tuning': (DOCS / 'tuning' / 'tuning-guide.md', DOCS / 'Pashupati-Tuning-Guide.pdf'),
}


def front_matter(text: str) -> tuple:
    """Return (metadata, body) of a source with a --- YAML --- header."""
    if not text.startswith('---\n'):
        return {}, text
    end = text.index('\n---\n', 4)
    return yaml.safe_load(text[4:end]) or {}, text[end + 5 :]


def number_headings(body: str) -> tuple:
    """Give h1 (1) and h2 (1.1) headings their numbers; return the HTML and the contents."""
    chapter, section, entries = 0, 0, []

    def number(match):
        nonlocal chapter, section
        level, attrs, text = int(match[1]), match[2], match[3]
        if 'unnumbered' in attrs:
            entries.append((level, '', re.search(r'id="([^"]+)"', attrs)[1], text))
            return match[0]
        if level == 1:
            chapter, section = chapter + 1, 0
            label = f'{chapter}'
        else:
            section += 1
            label = f'{chapter}.{section}'
        anchor = re.search(r'id="([^"]+)"', attrs)[1]
        entries.append((level, label, anchor, text))
        return f'<h{level}{attrs}><span class="num">{label}</span> {text}</h{level}>'

    body = re.sub(r'<h([12])([^>]*)>(.*?)</h\1>', number, body)
    return body, entries


def figures(body: str) -> str:
    """Turn a paragraph holding only an image into a figure with its alt text as caption."""

    def figure(match):
        tag = match[1]
        alt = re.search(r'alt="([^"]*)"', tag)
        css = re.search(r'class="([^"]*)"', tag)
        caption = f'<figcaption>{alt[1]}</figcaption>' if alt and alt[1] else ''
        return f'<figure class="{css[1] if css else ""}">{tag}{caption}</figure>'

    return re.sub(r'<p>\s*(<img [^>]+>)\s*</p>', figure, body)


def build(name: str):
    """Render one guide to its PDF."""
    source, target = GUIDES[name]
    meta, text = front_matter(source.read_text())
    text = text.replace('{{ parameter_reference }}', parameter_reference())
    body = markdown.markdown(
        text,
        extensions=[
            'tables',
            'attr_list',
            'admonition',
            'toc',
            'fenced_code',
            'md_in_html',
            'sane_lists',
        ],
        extension_configs={'toc': {'permalink': False}},
    )
    body, entries = number_headings(figures(body))
    contents = '\n'.join(
        f'<li class="toc{level}"><a href="#{anchor}"><span class="num">{label}</span> '
        f'{re.sub("<[^>]+>", "", text)}</a></li>'
        for level, label, anchor, text in entries
    )
    revisions = '\n'.join(
        f'<tr><td>{html.escape(str(v))}</td><td>{html.escape(str(d))}</td>'
        f'<td>{html.escape(str(n))}</td></tr>'
        for v, d, n in meta.get('revisions', [])
    )
    title = html.escape(meta.get('title', name))
    page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>{title}</title>
</head><body>
<section class="cover">
  <img class="logo" src="{(DOCS / 'logo.png').as_uri()}" alt="">
  <div class="brand">PASHUPATI</div>
  <h1 class="cover-title unnumbered">{title}</h1>
  <p class="cover-subtitle">{html.escape(meta.get('subtitle', ''))}</p>
  <p class="cover-version">Version {html.escape(str(meta.get('version', '')))} ·
     {html.escape(str(meta.get('date', '')))}</p>
  <p class="cover-for">{html.escape(meta.get('audience', ''))}</p>
</section>
<section class="front">
  <h2 class="front-title">Revision history</h2>
  <table class="revisions"><thead><tr><th>Version</th><th>Date</th><th>Changes</th></tr></thead>
  <tbody>{revisions}</tbody></table>
  <p class="front-note">{html.escape(meta.get('note', ''))}</p>
</section>
<section class="front contents">
  <h2 class="front-title">Contents</h2>
  <ul class="toc">{contents}</ul>
</section>
<main>{body}</main>
</body></html>"""
    css = (
        (Path(__file__).parent / 'style.css')
        .read_text()
        .replace('%HEADER%', meta.get('header', meta.get('title', '')))
    )
    fonts = FontConfiguration()  # loads the @font-face fonts of the stylesheet
    HTML(string=page, base_url=str(source.parent)).write_pdf(
        target,
        stylesheets=[CSS(string=css, base_url=str(Path(__file__).parent), font_config=fonts)],
        font_config=fonts,
    )
    print(f'{target.relative_to(ROOT)}: {len(entries)} sections')


if __name__ == '__main__':
    for guide in sys.argv[1:] or GUIDES:
        build(guide)
