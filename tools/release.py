#!/usr/bin/env python3
"""mabur releases: the drone<->GS contract, the version rule, release notes.

A release is tagged v0.MINOR.PATCH. MINOR moves when the drone<->GS contract
changes -- RC_VERSION, a CAP_* constant, or the drone parameter table (ids or
order) -- and PATCH moves otherwise. `check` refuses a tag whose PATCH moved
over a contract change. A removed or renamed config key also breaks a GS's
paired drone but leaves no constant behind; the tagger owns that one
(docs/releases.md).

  release.py contract <git-ref>          the contract at a commit, as JSON
  release.py check <tag>                 exit 1 if <tag> breaks the rule
  release.py notes --version V --sha S --build-info F --files DIR [--prev-tag T]
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys

RC_PROTO = "common/include/mabur/rc_proto.h"
CONFIG_PARAMS = "common/include/mabur/config_params.h"
TAG_RE = re.compile(r"^v(\d+)\.(\d+)\.(\d+)$")


def _parse(rc_text, params_text):
    m = re.search(r"constexpr\s+\w+\s+RC_VERSION\s*=\s*(\d+)", rc_text)
    if not m:
        raise ValueError("RC_VERSION not found in " + RC_PROTO)
    caps = {name: int(val, 0) for name, val in re.findall(
        r"constexpr\s+\w+\s+(\w*CAP_[A-Z0-9_]+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)", rc_text)}
    params = re.findall(r'^\s*\{"([A-Z0-9_]+)"', params_text, re.M)
    return {"rc_version": int(m.group(1)), "caps": caps, "params": params}


def contract(root):
    """The contract of the tree at `root`."""
    with open(os.path.join(root, RC_PROTO)) as f:
        rc = f.read()
    with open(os.path.join(root, CONFIG_PARAMS)) as f:
        params = f.read()
    return _parse(rc, params)


def contract_at(ref):
    """The contract of the commit `ref` in the current git repo."""
    show = lambda p: subprocess.run(["git", "show", f"{ref}:{p}"], check=True,
                                    capture_output=True, text=True).stdout
    return _parse(show(RC_PROTO), show(CONFIG_PARAMS))


def diff(prev, cur):
    """Human lines naming each contract change; empty when there is none."""
    out = []
    if prev["rc_version"] != cur["rc_version"]:
        out.append(f"RC_VERSION {prev['rc_version']} -> {cur['rc_version']}")
    added = sorted(set(cur["caps"]) - set(prev["caps"]))
    removed = sorted(set(prev["caps"]) - set(cur["caps"]))
    moved = sorted(k for k in set(prev["caps"]) & set(cur["caps"])
                   if prev["caps"][k] != cur["caps"][k])
    if added:
        out.append("caps added " + " ".join(added))
    if removed:
        out.append("caps removed " + " ".join(removed))
    if moved:
        out.append("caps renumbered " + " ".join(moved))
    if prev["params"] != cur["params"]:
        out.append(f"param rows {len(prev['params'])} -> {len(cur['params'])}"
                   + ("" if len(prev["params"]) != len(cur["params"]) else " (ids or order)"))
    return out


def check_tag(tag, prev_tag, changes):
    """None when `tag` follows the rule against `prev_tag`, else the reason."""
    m = TAG_RE.match(tag)
    if not m:
        return f"{tag}: release tags are vMAJOR.MINOR.PATCH (e.g. v0.1.0)"
    if prev_tag is None or not changes:
        return None
    major, minor, _ = (int(x) for x in m.groups())
    pmajor, pminor, _ = (int(x) for x in TAG_RE.match(prev_tag).groups())
    if (major, minor) == (pmajor, pminor):
        return (f"{tag} changes the drone<->GS contract since {prev_tag} "
                f"({'; '.join(changes)}) but only bumps the patch number. "
                f"Delete the tag and tag v{major}.{minor + 1}.0 instead.")
    return None


def previous_tag(tag):
    """The highest v* tag below `tag`, or None."""
    tags = subprocess.run(["git", "tag", "--list", "v*", "--sort=-v:refname"],
                          check=True, capture_output=True, text=True).stdout.split()
    key = lambda t: tuple(int(x) for x in TAG_RE.match(t).groups())
    below = [t for t in tags if TAG_RE.match(t) and key(t) < key(tag)]
    return below[0] if below else None


def notes(version, sha, info, cur, prev_tag, changes, files):
    """The release body, as markdown."""
    caps = " ".join(k.replace("CAP_", "") for k in sorted(cur["caps"], key=cur["caps"].get))
    lines = [f"mabur {version} (commit `{sha[:8]}`)", "",
             f"**Wire contract:** RC_VERSION {cur['rc_version']}, caps {caps}, "
             f"{len(cur['params'])} drone parameter rows"]
    if prev_tag is None:
        lines.append("This is the first release.")
    elif changes:
        lines.append(f"**Contract changed since {prev_tag}: update the GS with this drone.**")
        lines += [f"- {c}" for c in changes]
    else:
        lines.append(f"Contract unchanged since {prev_tag}: drop-in for any GS that flew {prev_tag}.")
    lines += ["", "**Built from**", f"- mabur `{sha}`"]
    for key in ("openipc", "devourer", "uboot"):
        lines.append(f"- {key} `{info.get(key, 'unknown')}`")
    for key in ("kernel_sha256", "toolchain_sha256", "ipctool_sha256"):
        lines.append(f"- {key.replace('_sha256', '')} sha256 `{info.get(key, 'unknown')}`")
    lines += ["", "| file | md5 |", "|---|---|"]
    lines += [f"| `{name}` | `{md5}` |" for name, md5 in sorted(files.items())]
    return "\n".join(lines) + "\n"


def _read_info(path):
    info = {}
    with open(path) as f:
        for line in f:
            if "=" in line:
                k, v = line.strip().split("=", 1)
                info[k] = v
    return info


def _md5s(directory):
    out = {}
    for name in sorted(os.listdir(directory)):
        p = os.path.join(directory, name)
        if os.path.isfile(p) and not name.endswith(".txt"):
            with open(p, "rb") as f:
                out[name] = hashlib.md5(f.read()).hexdigest()
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("contract")
    c.add_argument("ref")
    k = sub.add_parser("check")
    k.add_argument("tag")
    n = sub.add_parser("notes")
    n.add_argument("--version", required=True)
    n.add_argument("--sha", required=True)
    n.add_argument("--build-info", required=True)
    n.add_argument("--files", required=True)
    n.add_argument("--prev-tag")
    a = ap.parse_args(argv)

    if a.cmd == "contract":
        print(json.dumps(contract_at(a.ref), indent=1))
        return 0
    if a.cmd == "check":
        prev = previous_tag(a.tag) if TAG_RE.match(a.tag) else None
        changes = diff(contract_at(prev), contract_at(a.tag)) if prev else []
        err = check_tag(a.tag, prev, changes)
        if err:
            print("error: " + err, file=sys.stderr)
            return 1
        print(f"{a.tag}: ok (previous {prev or 'none'}; "
              f"{'; '.join(changes) if changes else 'contract unchanged'})")
        return 0
    cur = contract_at(a.sha)
    changes = diff(contract_at(a.prev_tag), cur) if a.prev_tag else []
    sys.stdout.write(notes(a.version, a.sha, _read_info(a.build_info), cur,
                           a.prev_tag, changes, _md5s(a.files)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
