//! `#[hot]` marks a function as a unit of work worth a span.
//!
//! It expands to
//! `#[cfg_attr(feature = "profile", tracing::instrument(level = "info", skip_all, ...))]`,
//! so the span exists only when the crate using it is built with its own
//! `profile` feature; otherwise the function is untouched. Arguments are
//! passed to `instrument` after `skip_all`:
//!
//! ```ignore
//! #[hot]
//! fn parse(src: &str) -> Ast { .. }
//!
//! #[hot(fields(net = %name, pops = tracing::field::Empty))]
//! fn route_net(name: &str) { .. }
//! ```
//!
//! The span is named after the function; its target is the module path,
//! which the flame graph and table put in front. A method names its type:
//! `#[hot(name = "Router::run")]`.

use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemFn, parse_macro_input};

#[proc_macro_attribute]
pub fn hot(attr: TokenStream, item: TokenStream) -> TokenStream {
    let f = parse_macro_input!(item as ItemFn);
    let args: proc_macro2::TokenStream = attr.into();
    let extra = if args.is_empty() { quote!() } else { quote!(, #args) };
    quote! {
        #[cfg_attr(feature = "profile", ::tracing::instrument(level = "info", skip_all #extra))]
        #f
    }
    .into()
}
