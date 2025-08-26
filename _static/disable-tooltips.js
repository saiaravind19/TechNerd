// Disable all tooltip behaviors
document.addEventListener('DOMContentLoaded', function() {
    // Remove title attributes that cause native tooltips
    const contactBtns = document.querySelectorAll('.contact-btn');
    contactBtns.forEach(btn => {
        btn.removeAttribute('title');
        btn.removeAttribute('data-bs-toggle');
        btn.removeAttribute('data-toggle');
        btn.removeAttribute('data-original-title');
        
        // Prevent any tooltip events
        btn.addEventListener('mouseenter', function(e) {
            e.stopPropagation();
        });
        
        btn.addEventListener('mouseleave', function(e) {
            e.stopPropagation();
            // Force hide any visible tooltips
            const tooltips = document.querySelectorAll('.tooltip, .bs-tooltip-top, .bs-tooltip-bottom, .bs-tooltip-left, .bs-tooltip-right');
            tooltips.forEach(tooltip => {
                tooltip.style.display = 'none';
                tooltip.style.opacity = '0';
                tooltip.style.visibility = 'hidden';
            });
        });
    });
    
    // Disable Bootstrap tooltip initialization
    if (window.bootstrap && window.bootstrap.Tooltip) {
        // Override tooltip constructor
        const originalTooltip = window.bootstrap.Tooltip;
        window.bootstrap.Tooltip = function() {
            return { show: function(){}, hide: function(){}, dispose: function(){} };
        };
    }
    
    // Force hide any existing tooltips on page load
    setTimeout(() => {
        const tooltips = document.querySelectorAll('.tooltip, .bs-tooltip-top, .bs-tooltip-bottom, .bs-tooltip-left, .bs-tooltip-right, [class*="tooltip"]');
        tooltips.forEach(tooltip => {
            tooltip.style.display = 'none';
            tooltip.style.opacity = '0';
            tooltip.style.visibility = 'hidden';
            tooltip.remove();
        });
    }, 100);
});

// Global click handler to hide tooltips
document.addEventListener('click', function() {
    const tooltips = document.querySelectorAll('.tooltip, .bs-tooltip-top, .bs-tooltip-bottom, .bs-tooltip-left, .bs-tooltip-right');
    tooltips.forEach(tooltip => {
        tooltip.style.display = 'none';
        tooltip.style.opacity = '0';
        tooltip.style.visibility = 'hidden';
    });
});
