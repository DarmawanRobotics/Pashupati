// The Pashupati documentation site: which Markdown of this repository becomes which page.
// Every page stays a file of the repository (the guides, READMEs); scripts/sync.mjs copies them.
const pkg = (name, text) => ({ src: `src/${name}/README.md`, dest: `packages/${name}`, text })

export default {
  title: 'Pashupati',
  description:
    'Autonomous patrol and inspection robot on the Dharma stack: teach and repeat on mapped ' +
    'sites, VLM inspection at stop points, crowd sensing, a handheld and the NETRA command center.',
  repo: 'DarmawanRobotics/Pashupati',
  branch: 'main',
  logo: 'logo.png',
  brand: { light: '#0f8a80', light2: '#14a396', dark: '#4fd1c5', dark2: '#7ee0d6', soft: 'rgba(79,209,197,.16)' },
  copyright: 'Apache-2.0 · DarmawanRobotics',
  sections: [
    {
      text: 'Guides',
      items: [
        {
          src: 'docs/manual/user-manual.md',
          dest: 'guides/user-manual',
          text: 'User Manual',
          numbered: true,
          pdf: 'Pashupati-User-Manual.pdf',
          frontmatter: { outline: [1, 2] },
        },
        {
          src: 'docs/tuning/tuning-guide.md',
          dest: 'guides/tuning-guide',
          text: 'Tuning Guide',
          numbered: true,
          pdf: 'Pashupati-Tuning-Guide.pdf',
          frontmatter: { outline: [1, 2] },
        },
      ],
    },
    {
      text: 'Reference',
      items: [
        { src: 'README.md', dest: 'reference/overview', text: 'Overview' },
        { src: 'src/pashupati_fleet/API.md', dest: 'reference/netra-api', text: 'NETRA API' },
        { src: 'src/pashupati_bridge/PROTOCOL.md', dest: 'reference/handheld-protocol', text: 'Handheld protocol' },
        { src: 'src/xtras/README.md', dest: 'reference/robot-packages', text: 'Robot packages' },
        { src: 'docs/README.md', dest: 'reference/documents', text: 'Documents' },
      ],
    },
    {
      text: 'Packages',
      items: [
        pkg('pashupati', 'pashupati'),
        pkg('pashupati_bridge', 'pashupati_bridge'),
        pkg('pashupati_fleet', 'pashupati_fleet'),
        pkg('pashupati_inspection', 'pashupati_inspection'),
        pkg('pashupati_utils', 'pashupati_utils'),
      ],
    },
    {
      text: 'Project',
      items: [
        { src: 'CONTRIBUTING.md', dest: 'project/contributing', text: 'Contributing' },
        { src: 'CHANGELOG.md', dest: 'project/changelog', text: 'Changelog' },
      ],
    },
  ],
  files: [
    { src: 'docs/Pashupati-User-Manual.pdf', as: 'Pashupati-User-Manual.pdf', text: 'User Manual' },
    { src: 'docs/Pashupati-Tuning-Guide.pdf', as: 'Pashupati-Tuning-Guide.pdf', text: 'Tuning Guide' },
  ],
  // the tuning guide's parameter reference, generated from the config files (as for the PDF)
  replacements: [
    { src: 'docs/tuning/tuning-guide.md', find: '{{ parameter_reference }}', command: 'python3 docs/pdf/params.py' },
  ],
}
