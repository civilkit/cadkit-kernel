// A small XML element tree: enough to write MJCF and to read it back (tests, MJCF import). No DTDs, no namespaces;
// comments, processing instructions and CDATA are skipped on read.

export interface XmlNode { tag: string; attrs: Record<string, string>; children: XmlNode[] }

export const el = (tag: string, attrs: Record<string, string | number | undefined> = {}, children: XmlNode[] = []): XmlNode => {
  const a: Record<string, string> = {};
  for (const [k, v] of Object.entries(attrs)) if (v !== undefined && v !== '') a[k] = String(v);
  return { tag, attrs: a, children };
};

const esc = (s: string): string => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

/** Indented XML text (two spaces). Raw fragments are nodes with tag `#raw` and the text in `attrs.text`. */
export function serialize(n: XmlNode, depth = 0): string {
  const pad = '  '.repeat(depth);
  if (n.tag === '#raw') return n.attrs.text.split('\n').map((l) => (l.trim() ? pad + l.trim() : '')).filter(Boolean).join('\n') + '\n';
  if (n.tag === '#comment') return `${pad}<!-- ${n.attrs.text.replace(/--/g, '- -')} -->\n`;
  const attrs = Object.entries(n.attrs).map(([k, v]) => ` ${k}="${esc(v)}"`).join('');
  if (!n.children.length) return `${pad}<${n.tag}${attrs}/>\n`;
  return `${pad}<${n.tag}${attrs}>\n${n.children.map((c) => serialize(c, depth + 1)).join('')}${pad}</${n.tag}>\n`;
}

const unesc = (s: string): string => s.replace(/&(lt|gt|quot|apos|amp|#\d+|#x[0-9a-fA-F]+);/g, (_, e: string) =>
  e === 'lt' ? '<' : e === 'gt' ? '>' : e === 'quot' ? '"' : e === 'apos' ? "'" : e === 'amp' ? '&' : String.fromCodePoint(e[1] === 'x' ? parseInt(e.slice(2), 16) : parseInt(e.slice(1), 10)));

/** Parse an XML document (or fragment with one root element). Throws on unbalanced tags. */
export function parse(text: string): XmlNode {
  const root: XmlNode = { tag: '#document', attrs: {}, children: [] };
  const stack: XmlNode[] = [root];
  const re = /<!--[\s\S]*?-->|<\?[\s\S]*?\?>|<!\[CDATA\[[\s\S]*?\]\]>|<!DOCTYPE[^>]*>|<\/\s*([\w:.-]+)\s*>|<([\w:.-]+)((?:\s+[\w:.-]+\s*=\s*(?:"[^"]*"|'[^']*'))*)\s*(\/?)>/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(text))) {
    if (m[1]) {
      const top = stack.pop();
      if (!top || top.tag !== m[1]) throw new Error(`xml: </${m[1]}> does not close <${top?.tag}>`);
    } else if (m[2]) {
      const attrs: Record<string, string> = {};
      for (const a of (m[3] ?? '').matchAll(/([\w:.-]+)\s*=\s*(?:"([^"]*)"|'([^']*)')/g)) attrs[a[1]] = unesc(a[2] ?? a[3] ?? '');
      const node: XmlNode = { tag: m[2], attrs, children: [] };
      stack[stack.length - 1].children.push(node);
      if (!m[4]) stack.push(node);
    }
  }
  if (stack.length !== 1) throw new Error(`xml: <${stack[stack.length - 1].tag}> is not closed`);
  if (root.children.length !== 1) throw new Error('xml: expected one root element');
  return root.children[0];
}

/** Every descendant (depth first) with the tag. */
export function findAll(n: XmlNode, tag: string, out: XmlNode[] = []): XmlNode[] {
  for (const c of n.children) { if (c.tag === tag) out.push(c); findAll(c, tag, out); }
  return out;
}
export const child = (n: XmlNode, tag: string): XmlNode | undefined => n.children.find((c) => c.tag === tag);
export const nums = (s: string | undefined): number[] => (s ?? '').trim().split(/\s+/).filter(Boolean).map(Number);
